#include "pico/stdlib.h"
#include "hardware/spi.h"

constexpr auto CPU_FREQ = 125'000'000;

consteval size_t spin_cycles(const uint64_t cpu_freq, const uint64_t ns) {
	return static_cast<size_t>(cpu_freq * ns / 1'000'000'000);
}

template <size_t ns>
void spin_delay() {
	constexpr auto cycles = spin_cycles(CPU_FREQ, ns) / 2;
	for (size_t i = 0; i < cycles; i++) {
		__asm__ volatile("nop");
	}
}

// https://github.com/raspberrypi/pico-examples/blob/master/spi/bme280_spi/bme280_spi.c
[[noreturn]]
int main() {
	spi_init(spi_default, 1'000'000);
	constexpr auto d = spin_cycles(CPU_FREQ, 3'000) / 2;
	for (;;) {}
}