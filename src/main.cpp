#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/spi.h>
#include <cstdio>
#include <etl/span.h>
#include "logging.h"
#include "common.hpp"
#include "ads1292r.hpp"

#include <pico/stdio_usb.h>
#include <pico/binary_info/code.h>

constexpr auto NS_PER_SEC = 1'000'000'000;
constexpr auto MS_PER_NS  = 1'000'000;
constexpr auto CPU_FREQ   = 125'000'000;

consteval size_t spin_cycles(const uint64_t cpu_freq, const uint64_t ns) {
	return static_cast<size_t>(cpu_freq * ns / 1'000'000'000);
}

/**
 * @brief Spin delay for a given number of nanoseconds by keeping the CPU busy
 * @tparam ns Number of nanoseconds to delay
 * @sa https://forums.raspberrypi.com/viewtopic.php?t=333928
 */
template <size_t ns>
void spin_block() {
	constexpr auto total = spin_cycles(CPU_FREQ, ns) / 3;
	asm volatile(
		"mov  r0, %[count]\n"           // 1 cycle
		"spin_loop: subs  r0, r0, #1\n" // 1 cycle
		"bne   spin_loop\n"             // 2 cycles if loop taken, 1 if not
		:
		: [count] "r"(total)
		: "r0");
}

namespace ADS1292R {
constexpr auto TAG        = "ads1292r";
constexpr auto spi_pre_cb = []() {
	gpio_put(common::pin::CS, false);
};
constexpr auto spi_post_cb = []() {
	// wait for 4us to satisfy the timing requirements of the ADS1292R
	sleep_us(8);
	gpio_put(common::pin::CS, true);
};

constexpr auto send_cmd = [](spi_inst_t *spi, const uint8_t cmd) {
	constexpr auto len = 1;
	uint8_t tx_buf[len];
	tx_buf[0] = cmd;

	spi_pre_cb();
	const auto n = spi_write_blocking(spi, tx_buf, len);
	spi_post_cb();
	assert(n == len);
};

constexpr auto read = [](spi_inst_t *spi, const uint8_t reg) {
	const uint8_t reg_addr = (reg & 0x7f) | ADS1292R::RREG;
	constexpr auto len     = 3;
	uint8_t tx_buf[len];
	// r rrrr = starting register address for read and write opcodes.
	tx_buf[0] = reg_addr;
	// n nnnn = number of registers to be read or written – 1.
	// For example, to read or write three registers,
	// set n nnnn = 0 (0010).
	tx_buf[1] = 0;
	uint8_t rx_buf[len];

	spi_pre_cb();
	const auto n = spi_write_read_blocking(spi, tx_buf, rx_buf, len);
	spi_post_cb();
	assert(n == len);
	return rx_buf[2];
};

constexpr auto write = [](spi_inst_t *spi, const uint8_t reg, const uint8_t data) {
	const uint8_t reg_addr = (reg & 0x7f) | ADS1292R::WREG;
	constexpr auto len     = 3;
	uint8_t tx_buf[len];
	tx_buf[0] = reg_addr;
	tx_buf[1] = 0;
	tx_buf[2] = data;

	spi_pre_cb();
	const auto n = spi_write_blocking(spi, tx_buf, len);
	spi_post_cb();
	assert(n == len);
};

constexpr auto checked_write = [](spi_inst_t *spi, const uint8_t reg, const uint8_t data) {
	write(spi, reg, data);
	const auto read_data = read(spi, reg);
	if (read_data != data) {
		LOGE(TAG, "expected %d, got %d", data, read_data);
		return false;
	}
	return true;
};

constexpr auto batch_read_data = [](spi_inst_t *spi, etl::span<uint8_t> buf) {
	constexpr auto len = 1 + ADS1292R::DATA_BYTES;
	if (buf.size() < len) {
		return 0;
	}
	uint8_t tx_buf[len]{};
	tx_buf[0]    = ADS1292R::RDATA;
	const auto n = spi_write_read_blocking(spi, tx_buf, buf.data(), len);
	assert(n == len);
	// discard the first byte
	// https://en.cppreference.com/w/cpp/algorithm/copy
	// If d_first is in `[first, last)`, the behavior is undefined.
	etl::copy(buf.begin() + 1, buf.end(), buf.begin());
	return ADS1292R::DATA_BYTES;
};

enum class DataStart : uint8_t {
	STOP  = 0x00,
	START = 0x01,
};

/**
 * @brief Set the START pin to start or stop data transfer
 */
constexpr auto ctrl_data_transfer = [](spi_inst_t *spi, const DataStart start) {
	if (start == DataStart::START) {
		gpio_put(common::pin::START, true);
		send_cmd(spi, ADS1292R::START);
	} else {
		gpio_put(common::pin::START, false);
		send_cmd(spi, ADS1292R::STOP);
	}
};

constexpr auto init = [] -> spi_inst_t * {
	gpio_init(common::pin::CS);
	gpio_init(common::pin::START);
	gpio_init(common::pin::RESET);
	gpio_init(common::pin::DRDY);

	gpio_set_dir(common::pin::CS, GPIO_OUT);
	gpio_set_dir(common::pin::START, GPIO_OUT);
	gpio_set_dir(common::pin::RESET, GPIO_OUT);
	gpio_set_dir(common::pin::DRDY, GPIO_IN);

	gpio_set_function(common::pin::MISO, GPIO_FUNC_SPI);
	gpio_set_function(common::pin::MOSI, GPIO_FUNC_SPI);
	gpio_set_function(common::pin::SCK, GPIO_FUNC_SPI);
	bi_decl(bi_3pins_with_func(common::pin::MISO, common::pin::MOSI, common::pin::SCK, GPIO_FUNC_SPI));
	bi_decl(bi_1pin_with_name(common::pin::CS, "CS"));

	auto &spi                   = *spi_default;
	constexpr auto spi_baudrate = 1'000'000;
	spi_init(&spi, spi_baudrate);
	// SPI mode 1
	// https://www.analog.com/en/resources/analog-dialogue/articles/introduction-to-spi-interface.html
	spi_set_format(&spi, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

	return &spi;
};

constexpr auto check_id = [](spi_inst_t *spi) {
	constexpr auto correct_id = id_t::ADS1292R_ID;
	constexpr auto TAG        = "ads1292r";
	auto retry                = 0;
	constexpr auto max_retry  = 5;
	while (retry < max_retry) {
		const auto id = read(spi, id_t::ID_ADDRESS);
		if (id == correct_id) {
			LOGD(TAG, "found ADS1292R");
			return true;
		}
		LOGW(TAG, "ID expected 0x%" PRIx8 " but got 0x%" PRIx8, correct_id, id);

		retry++;
		sleep_ms(10);
	}
	return false;
};
} // namespace ADS1292R

// Communication Device Class
namespace cdc1 {
constexpr auto write = cdc_1_usb_out_chars;
constexpr auto flush = cdc_1_out_flush;

/**
 * @brief Print formatted data to the USB CDC
 * @tparam N buffer size
 * @param fmt format string
 * @param ... arguments
 * @return number of characters printed
 */
template <size_t N = 128>
int printf(const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	char buf[N];
	const auto n = vsnprintf(buf, N, fmt, args);
	cdc1::write(buf, n);
	va_end(args);
	return n;
}
} // namespace cdc1

// https://github.com/raspberrypi/pico-examples/blob/master/gpio/hello_gpio_irq/hello_gpio_irq.c
// https://github.com/raspberrypi/pico-examples/blob/master/spi/bme280_spi/bme280_spi.c
[[noreturn]]
int main() {
	using namespace common;
	constexpr auto TAG = "main";
	// on dual virtual port
	// https://github.com/hathach/tinyusb/tree/master/examples/device/cdc_dual_ports
	// https://www.reddit.com/r/embedded/comments/130xlw9/usb_cdc_multiple_virtual_com_ports
	// https://github.com/Noltari/pico-uart-bridge/blob/master/usb-descriptors.c
	// https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2_common/pico_stdio_usb
	// https://www.pschatzmann.ch/home/2021/02/19/tinyusb-a-simple-tutorial/
	stdio_usb_init();

	gpio_init(pin::BUILT_IN_LED);
	gpio_set_dir(pin::BUILT_IN_LED, GPIO_OUT);
	gpio_put(pin::BUILT_IN_LED, true);
	auto &spi = *ADS1292R::init();
	gpio_put(pin::RESET, false);
	sleep_ms(1);
	gpio_put(pin::RESET, true);
	sleep_ms(1);
	ADS1292R::ctrl_data_transfer(&spi, ADS1292R::DataStart::STOP);

	static auto n = 0;
	for (;;) {
		LOGI(TAG, "loop %d", n);
		cdc1::printf("from cdc1 %d\r\n", n);
		sleep_ms(500);
		n++;
		gpio_put(pin::BUILT_IN_LED, n % 2);
	}
	return 0;
}