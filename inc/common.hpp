//
// Created by cross on 8/25/2024.
//

#ifndef COMMON_H
#define COMMON_H

namespace common {
namespace pin {
	constexpr auto BUILT_IN_LED = 25;
	constexpr auto SCK          = PICO_DEFAULT_SPI_SCK_PIN;
	constexpr auto MOSI         = PICO_DEFAULT_SPI_TX_PIN;
	constexpr auto MISO         = PICO_DEFAULT_SPI_RX_PIN;
	constexpr auto CS           = PICO_DEFAULT_SPI_CSN_PIN;
	constexpr auto DRDY         = 6;
	constexpr auto START        = 7;
	constexpr auto RESET        = 8;
} // namespace pin
constexpr auto SPI_CONTROLLER_ID = PICO_DEFAULT_SPI;
} // namespace common

#endif // COMMON_H
