#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/spi.h>
#include <cstdio>

constexpr auto LED_PIN    = 25;
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

// https://github.com/raspberrypi/pico-examples/blob/master/spi/bme280_spi/bme280_spi.c
[[noreturn]]
int main() {
	stdio_init_all();
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	for (;;) {
		printf("Hello, world!\n");
		gpio_put(LED_PIN, true);
		sleep_ms(1000);
		gpio_put(LED_PIN, false);
		sleep_ms(1000);
	}
}
