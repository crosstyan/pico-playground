//
// Created by Kurosu Chan on 2024/7/15.
//

#ifndef ADS1292R_H
#define ADS1292R_H

#include <cassert>
#include <cstdint>

namespace ADS1292R {
#define PACK __attribute__((packed))
// Register Read Commands
//
// Use RREG|ADDRESS or WREG|ADDRESS to read or write one register.

/**
 * r rrrr=要读、写的寄存器首地址  // n nnnn=要读写的寄存器数量
 *
 * Read n nnnn registers starting at address r rrrr,
 * first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2).
 *
 * 读取  001r rrrr(首字节) 000n nnnn(2字节)
 */
constexpr uint8_t RREG = 0x20;
/**
 * 写入  010r rrrr(首字节) 000n nnnn(2字节)
 *
 * Write n nnnn registers starting at address r rrrr,
 * first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)
 */
constexpr uint8_t WREG = 0x40;

constexpr uint8_t START = 0x08; // Start/restart (synchronize) conversions 启动或转换
constexpr uint8_t STOP  = 0x0A; // Stop conversion 停止转换

// ADS1292R 具有呼吸和导联脱落检测的功能
// ID
constexpr uint8_t ADS1292_DEVICE = 0x73; // 注意1292芯片不能使用呼吸相关的功能，会导致数据不正常

// 1 CONFIG1
constexpr uint8_t DATA_RATE = 0x02; // 由于时钟原因，采样率会有误差

// 2 CONFIG2
constexpr uint8_t PDB_LOFF_COMP = 1; // 导联脱落比较器
constexpr uint8_t PDB_REFBUF    = 1; // 基准电压缓冲器
constexpr uint8_t VREF          = 0; // 设置内部参考电压
constexpr uint8_t CLK_EN        = 0; // 使用内部时钟时，CLK引脚是否输出时钟信号
constexpr uint8_t INT_TEST      = 0; // 是否打开内部测试信号

// 4 5 CHSET
constexpr uint8_t CNNNLE1_POWER = 0; // 通道电源
constexpr uint8_t CNNNLE1_GAIN  = 2; // 增益
constexpr uint8_t CNNNLE1_MUX   = 0; // 输入方式
constexpr uint8_t CNNNLE2_POWER = 0;
constexpr uint8_t CNNNLE2_GAIN  = 0;
constexpr uint8_t CNNNLE2_MUX   = 0;

// 6 RLD_SENS
constexpr uint8_t PDB_RLD        = 1; // RLD缓冲电源
constexpr uint8_t RLD_LOFF_SENSE = 0; // RLD导联脱落功能（测试的时候发现右腿导联脱落检测和右腿驱动输出好像不能同时工作）
constexpr uint8_t RLD2N          = 1; // 通道的右腿驱动输出
constexpr uint8_t RLD2P          = 1;
constexpr uint8_t RLD1N          = 0; // 呼吸通道不需要右腿驱动
constexpr uint8_t RLD1P          = 0;

// 7 LOFF_SENS
constexpr uint8_t FLIP2  = 0; // 这个位用于控制导联脱落检测通道1的电流的方向
constexpr uint8_t FLIP1  = 0; // 这个位用于控制导联脱落检测通道2的电流的方向
constexpr uint8_t LOFF2N = 1; // 通道导联脱落检测功能
constexpr uint8_t LOFF2P = 1;
constexpr uint8_t LOFF1N = 0; // 呼吸通道不需要导联脱落检测
constexpr uint8_t LOFF1P = 0;

// 9 RSP1
constexpr uint8_t RESP_DEMOD_EN1 = 1;    // 启用通道1解调电路
constexpr uint8_t RESP_MOD_EN    = 1;    // 启用通道1调制电路
constexpr uint8_t RESP_PH        = 0x0D; // 控制解调信号的相位 135
constexpr uint8_t RESP_CTRL      = 0;    // 内部呼吸内部时钟

// 10 RSP2
constexpr uint8_t CALIB      = 0; // 通道偏移校正 //运行过程中如果改变增益需要打开通道偏移校正，并执行 OFFSETCAL 指令
constexpr uint8_t FREQ       = 0; // 呼吸调制频率 64K/32K
constexpr uint8_t RLDREF_INT = 1; // RLD参考 内部馈电  右腿驱动的参考电压可以选择内部产生(AVDD + AVSS) / 2，也可以选择外部提供

// ADS1292R命令定义
// 系统命令
constexpr uint8_t WAKEUP    = 0x02; // 从待机模式唤醒
constexpr uint8_t STANDBY   = 0x04; // 进入待机模式
constexpr uint8_t RESET     = 0x06; // 复位ADS1292R
constexpr uint8_t OFFSETCAL = 0x1A; // 通道偏移校准

// 数据读取命令
constexpr uint8_t RDATAC = 0x10; // 启用连续的数据读取模式 Read Data Continuous mode (default)
constexpr uint8_t SDATAC = 0x11; // 停止连续的数据读取模式 Stop Read Data Continuously mode
constexpr uint8_t RDATA  = 0x12; // Read data by command; supports multiple read back

// 寄存器设置
// ID
constexpr uint8_t DEVICE_ID_ADS1292  = 0x53;
constexpr uint8_t DEVICE_ID_ADS1292R = 0x73;

// CONFIG1
constexpr uint8_t DATA_RATE_125SPS = 0x00; // 采样率
constexpr uint8_t DATA_RATE_250SPS = 0x01;
constexpr uint8_t DATA_RATE_500SPS = 0x02;
constexpr uint8_t DATA_RATE_1kSPS  = 0x03;
constexpr uint8_t DATA_RATE_2kSPS  = 0x04;
constexpr uint8_t DATA_RATE_4kSPS  = 0x05;
constexpr uint8_t DATA_RATE_8kSPS  = 0x06;

// CONFIG2
constexpr uint8_t PDB_LOFF_COMP_OFF = 0; // 导联脱落比较器掉电（默认）
constexpr uint8_t PDB_LOFF_COMP_ON  = 1;
constexpr uint8_t PDB_REFBUF_OFF    = 0; // 基准电压缓冲器掉电（默认）
constexpr uint8_t PDB_REFBUF_ON     = 1;
constexpr uint8_t VREF_242V         = 0; // 内部参考电压2.42V（默认）
constexpr uint8_t VREF_4V           = 1; // 内部参考电压4.033V
constexpr uint8_t CLK_EN_OFF        = 0; // 振荡器时钟输出禁用（默认）
constexpr uint8_t CLK_EN_ON         = 1;
constexpr uint8_t INT_TEST_OFF      = 0; // 关闭内部测试信号（默认）
constexpr uint8_t INT_TEST_ON       = 1;

// CHSET
constexpr uint8_t PD_ON             = 0; // 通道正常运行（默认）
constexpr uint8_t PD_OFF            = 1;
constexpr uint8_t GAIN_6            = 0; // 增益6（默认）
constexpr uint8_t GAIN_1            = 1;
constexpr uint8_t GAIN_2            = 2;
constexpr uint8_t GAIN_3            = 3;
constexpr uint8_t GAIN_4            = 4;
constexpr uint8_t GAIN_8            = 5;
constexpr uint8_t GAIN_12           = 6;
constexpr uint8_t MUX_Normal_input  = 0; // 普通电极输入（默认）
constexpr uint8_t MUX_input_shorted = 1; // 输入短路
constexpr uint8_t MUX_Test_signal   = 5; // 测试信号输入
constexpr uint8_t MUX_RLD_DRP       = 6;
constexpr uint8_t MUX_RLD_DRM       = 7;
constexpr uint8_t MUX_RLD_DRPM      = 8;
constexpr uint8_t MUX_RSP_IN3P      = 9; // 呼吸道通道1

// RLD_SENS
constexpr uint8_t PDB_RLD_OFF        = 0; // （默认）
constexpr uint8_t PDB_RLD_ON         = 1;
constexpr uint8_t RLD_LOFF_SENSE_OFF = 0; // （默认）
constexpr uint8_t RLD_LOFF_SENSE_ON  = 1;
constexpr uint8_t RLD_CANNLE_OFF     = 0; // （默认）
constexpr uint8_t RLD_CANNLE_ON      = 1;

// LOFF_SENS
constexpr uint8_t FLIP2_OFF       = 0; // （默认）
constexpr uint8_t FLIP2_ON        = 1;
constexpr uint8_t FLIP1_OFF       = 0; // （默认）
constexpr uint8_t FLIP1_ON        = 1;
constexpr uint8_t LOFF_CANNLE_OFF = 0; // （默认）
constexpr uint8_t LOFF_CANNLE_ON  = 1;

// RSP1
constexpr uint8_t RESP_DEMOD_OFF           = 0; // （默认）
constexpr uint8_t RESP_DEMOD_ON            = 1;
constexpr uint8_t RESP_MOD_OFF             = 0; // （默认）
constexpr uint8_t RESP_MOD_ON              = 1;
constexpr uint8_t RESP_CTRL_CLOCK_INTERNAL = 0;
constexpr uint8_t RESP_CTRL_CLOCK_EXTERNAL = 1;

// RSP2
constexpr uint8_t CALIB_OFF             = 0; // （默认）
constexpr uint8_t CALIB_ON              = 1;
constexpr uint8_t FREQ_32K              = 0; // （默认）
constexpr uint8_t FREQ_64K              = 1;
constexpr uint8_t RLDREF_INT_EXTERN     = 0; // 外部馈电RLDREF
constexpr uint8_t RLDREF_INT_INTERNALLY = 1; // 内部

struct PACK id_t {
	static constexpr auto ID_ADDRESS           = 0x00;
	static constexpr uint8_t PRI_REV_ID_OFFSET = 5;
	static constexpr auto ADS1292R_ID          = 0x73;
	// MSB

	uint8_t pri_rev_id : 3;
	uint8_t must_1 : 1;
	uint8_t must_0 : 2;
	uint8_t minor_rev_id : 2;
	// LSB

	static constexpr uint8_t PRI_REV_ADS1x9x  = 0b010;
	static constexpr uint8_t PRI_REV_ADS1292R = 0b011;

	static constexpr uint8_t MINOR_REV_ADS1191   = 0b00;
	static constexpr uint8_t MINOR_REV_ADS1192   = 0b01;
	static constexpr uint8_t MINOR_REV_ADS1291   = 0b10;
	static constexpr uint8_t MINOR_REV_ADS1292_R = 0b11;

	static id_t from_byte(uint8_t byte) {
		id_t ret;
		ret.pri_rev_id   = (byte >> PRI_REV_ID_OFFSET) & 0b111;
		ret.must_1       = (byte >> 4) & 1;
		ret.must_0       = (byte >> 2) & 0b11;
		ret.minor_rev_id = byte & 0b11;
		assert(ret.must_1 == 1);
		assert(ret.must_0 == 0);
		return ret;
	}
};

// 配置寄存器 1
struct PACK config_1_t {
	static constexpr auto CONFIG_1_ADDRESS = 0x01;
	// MSB
	bool single_shot : 1;
	uint8_t must_0 : 4 = 0;
	uint8_t data_rate : 3;
	// LSB

	static constexpr uint8_t SINGLE_SHOT_OFFSET = 7;
	static constexpr uint8_t DR_125_SPS         = 0b000;
	static constexpr uint8_t DR_250_SPS         = 0b001;
	static constexpr uint8_t DR_500_SPS         = 0b010; // default
	static constexpr uint8_t DR_1k_SPS          = 0b011;
	static constexpr uint8_t DR_2k_SPS          = 0b100;
	static constexpr uint8_t DR_4k_SPS          = 0b101;
	static constexpr uint8_t DR_8k_SPS          = 0b110;
	static constexpr uint8_t DR_DO_NOT_USE      = 0b111;

	uint8_t to_byte() const {
		uint8_t ret = 0;
		ret |= (single_shot << SINGLE_SHOT_OFFSET);
		ret |= (data_rate & 0b111);
		return ret;
	}
	static config_1_t from_byte(uint8_t byte) {
		config_1_t ret;
		ret.single_shot = (byte >> SINGLE_SHOT_OFFSET) & 1;
		ret.must_0      = 0;
		ret.data_rate   = byte & 0b111;
		return ret;
	}
};

// 配置寄存器 2
struct PACK config_2_t {
	static constexpr auto CONFIG_2_ADDRESS = 0x02;
	// MSB

	bool must_one : 1 = true;
	/// This bit powers down the lead-off comparators.
	///   - 0: power down (default)
	///   - 1: power up
	bool pdb_loff_comp : 1;
	/// This bit powers down the internal reference buffer so that the external reference can be used.
	///   - 0: power down (default)
	///   - 1: power up
	bool pdb_refbuf : 1;
	/// This bit chooses between 2.42-V and 4.033-V reference.
	///   - 0: 2.42V (default)
	///   - 1: 4.033V
	bool vref_4v : 1;
	/// This bit determines if the internal oscillator signal is connected to the CLK pin when an internal oscillator is used.
	///	- 0: disable
	///	- 1: enable
	bool clk_en : 1;
	bool must_0 : 1 = false;
	/// This bit determines whether the test signal is turned on or off.
	///   - 0: disable
	///   - 1: enable; amplitude=±(VREFP - VREFN) / 2400
	bool int_test : 1;
	/// This bit determines the test frequency.
	/// - 0: at DC (default)
	/// - 1: Square wave @ 1Hz
	bool test_freq : 1;
	// LSB

	uint8_t to_byte() const {
		uint8_t ret = 0;
		ret |= (1 << 7);
		ret |= (pdb_loff_comp << 6);
		ret |= (pdb_refbuf << 5);
		ret |= (vref_4v << 4);
		ret |= (clk_en << 3);
		ret |= (0 << 2);
		ret |= (int_test << 1);
		ret |= (test_freq << 0);
		return ret;
	}

	static config_2_t from_byte(uint8_t byte) {
		config_2_t ret;
		ret.must_one      = (byte >> 7) & 1;
		ret.pdb_loff_comp = (byte >> 6) & 1;
		ret.pdb_refbuf    = (byte >> 5) & 1;
		ret.vref_4v       = (byte >> 4) & 1;
		ret.clk_en        = (byte >> 3) & 1;
		ret.must_0        = (byte >> 2) & 1;
		ret.int_test      = (byte >> 1) & 1;
		ret.test_freq     = (byte >> 0) & 1;
		assert(ret.must_one == 1);
		assert(ret.must_0 == 0);
		return ret;
	}
};

// 导联脱落控制寄存器
// Lead-Off Control Register
struct PACK loff_t {
	static constexpr uint8_t LOFF_ADDRESS = 0x03;

	static constexpr uint8_t COMP_TH_OFFSET   = 5;
	static constexpr uint8_t ILEAD_OFF_OFFSET = 2;
	static constexpr uint8_t FLEAD_OFF_OFFSET = 0;

	static constexpr uint8_t COMP_TH_POS_95   = 0b000;
	static constexpr uint8_t COMP_TH_POS_92_5 = 0b001;
	static constexpr uint8_t COMP_TH_POS_90   = 0b010;
	static constexpr uint8_t COMP_TH_POS_87_5 = 0b011;
	static constexpr uint8_t COMP_TH_POS_85   = 0b100;
	static constexpr uint8_t COMP_TH_POS_80   = 0b101;
	static constexpr uint8_t COMP_TH_POS_75   = 0b110;
	static constexpr uint8_t COMP_TH_POS_70   = 0b111;

	static constexpr uint8_t COMP_TH_NEG_5    = 0b000;
	static constexpr uint8_t COMP_TH_NEG_7_5  = 0b001;
	static constexpr uint8_t COMP_TH_NEG_10   = 0b010;
	static constexpr uint8_t COMP_TH_NEG_12_5 = 0b011;
	static constexpr uint8_t COMP_TH_NEG_15   = 0b100;
	static constexpr uint8_t COMP_TH_NEG_20   = 0b101;
	static constexpr uint8_t COMP_TH_NEG_25   = 0b110;
	static constexpr uint8_t COMP_TH_NEG_30   = 0b111;

	static constexpr uint8_t ILEAD_OFF_6NA  = 0b00;
	static constexpr uint8_t ILEAD_OFF_22NA = 0b01;
	static constexpr uint8_t ILEAD_OFF_6UA  = 0b10;
	static constexpr uint8_t ILEAD_OFF_22UA = 0b11;

	// MSB
	uint8_t comp_th : 3;    // Lead-off comparator threshold
	uint8_t must_1 : 1 = 1; // Must be set to '1'
	uint8_t ilead_off : 2;  // Lead-off current magnitude
	uint8_t must_0 : 1 = 0; // Must be set to '0'
	/// This bit selects ac or dc lead-off.
	///   - 0 = At dc lead-off detect (default)
	///   - 1 = At ac lead-off detect at fDR / 4 (500 Hz for an 2-kHz output rate)
	bool flead_off : 1; // Lead-off frequency
	// LSB

	uint8_t to_byte() const {
		uint8_t ret = 0;
		ret |= (comp_th << COMP_TH_OFFSET);
		ret |= (1 << 4); // Reserved bit must be set to '1'
		ret |= (ilead_off << ILEAD_OFF_OFFSET);
		// Reserved bit at offset 1 is already 0
		ret |= flead_off;
		return ret;
	}

	static loff_t from_byte(uint8_t byte) {
		loff_t ret;
		ret.comp_th   = (byte >> COMP_TH_OFFSET) & 0x7;
		ret.must_1    = 1; // Always set to 1
		ret.ilead_off = (byte >> ILEAD_OFF_OFFSET) & 0x3;
		ret.must_0    = 0; // Always set to 0
		ret.flead_off = byte & 1;
		return ret;
	}
};

// 通道设置寄存器
// Channel settings, x = 1 or 2
struct PACK chxset_t {
	static constexpr uint8_t CH1SET_ADDRESS = 0x04;
	static constexpr uint8_t CH2SET_ADDRESS = 0x05;

	static constexpr uint8_t PD_OFFSET   = 7;
	static constexpr uint8_t GAIN_OFFSET = 4;
	static constexpr uint8_t GAIN_6      = 0b000;
	static constexpr uint8_t GAIN_1      = 0b001;
	static constexpr uint8_t GAIN_2      = 0b010;
	static constexpr uint8_t GAIN_3      = 0b011;
	static constexpr uint8_t GAIN_4      = 0b100;
	static constexpr uint8_t GAIN_8      = 0b101;
	static constexpr uint8_t GAIN_12     = 0b110;

	// 0000 = Normal electrode input (default)
	// 0001 = Input shorted (for offset measurements)
	// 0010 = RLD_MEASURE
	// 0011 = MVDD(2) for supply measurement
	// 0100 = Temperature sensor
	// 0101 = Test signal
	// 0110 = RLD_DRP (positive input is connected to RLDIN)
	// 0111 = RLD_DRM (negative input is connected to RLDIN)
	// 1000 = RLD_DRPM (both positive and negative inputs are connected to RLDIN)
	// 1001 = Route IN3P and IN3N to channel 1 inputs
	// 1010 = Reserved
	static constexpr uint8_t MUX_OFFSET                 = 0;
	static constexpr uint8_t MUX_NORMAL_ELECTRODE_INPUT = 0b0000; // default
	static constexpr uint8_t MUX_INPUT_SHORTED          = 0b0001;
	static constexpr uint8_t MUX_RLD_MEASURE            = 0b0010;
	static constexpr uint8_t MUX_MVDD                   = 0b0011;
	static constexpr uint8_t MUX_TEMPERATURE_SENSOR     = 0b0100;
	static constexpr uint8_t MUX_TEST_SIGNAL            = 0b0101;
	static constexpr uint8_t MUX_RLD_DRP                = 0b0110;
	static constexpr uint8_t MUX_RLD_DRM                = 0b0111;
	static constexpr uint8_t MUX_RLD_DRPM               = 0b1000;
	static constexpr uint8_t MUX_ROUTE_IN3P_IN3N        = 0b1001;
	static constexpr uint8_t MUX_RESERVED               = 0b1010;

	static constexpr bool PD_POWER_ON  = false;
	static constexpr bool PD_POWER_OFF = true;

	// MSB
	bool pd : 1; // 0: normal operation, 1: power-down
	uint8_t gain : 3;
	uint8_t mux : 4;
	// LSB

	uint8_t to_byte() const {
		uint8_t ret = 0;
		ret |= (pd << PD_OFFSET);
		ret |= (gain << GAIN_OFFSET);
		ret |= (mux & 0b1111);
		return ret;
	}

	static chxset_t from_byte(uint8_t byte) {
		chxset_t ret;
		ret.pd   = (byte >> PD_OFFSET) & 1;
		ret.gain = (byte >> GAIN_OFFSET) & 0b111;
		ret.mux  = byte & 0b1111;
		return ret;
	}
};

// 右腿驱动选择寄存器
// Right Leg Drive Sense Selection
struct PACK rld_sens_t {
	static constexpr uint8_t RLD_SENS_ADDRESS = 0x06;

	static constexpr uint8_t CHOP_OFFSET          = 6;
	static constexpr uint8_t PDB_RLD_OFFSET       = 5;
	static constexpr uint8_t RLD_LOFF_SENS_OFFSET = 4;
	static constexpr uint8_t RLD2N_OFFSET         = 3;
	static constexpr uint8_t RLD2P_OFFSET         = 2;
	static constexpr uint8_t RLD1N_OFFSET         = 1;
	static constexpr uint8_t RLD1P_OFFSET         = 0;

	static constexpr uint8_t CHOP_FMOD_16  = 0b00;
	static constexpr uint8_t CHOP_RESERVED = 0b01;
	static constexpr uint8_t CHOP_FMOD_2   = 0b10;
	static constexpr uint8_t CHOP_FMOD_4   = 0b11;

	// MSB

	/// Chop frequency
	///
	/// These bits determine PGA chop frequency
	uint8_t chop : 2;
	/// RLD buffer power
	///
	/// This bit determines the RLD buffer power state.
	bool pdb_rld : 1;
	/// RLD lead-off sense function
	///
	/// This bit enables the RLD lead-off sense function.
	bool rld_loff_sens : 1;
	/// Channel 2 RLD negative inputs
	///
	/// This bit controls the selection of negative inputs from channel 2 for right leg drive derivation.
	bool rld2n : 1;
	/// Channel 2 RLD positive inputs
	///
	/// This bit controls the selection of positive inputs from channel 2 for right leg drive derivation.
	bool rld2p : 1;
	/// Channel 1 RLD negative inputs
	///
	/// This bit controls the selection of negative inputs from channel 1 for right leg drive derivation.
	bool rld1n : 1;
	/// Channel 1 RLD positive inputs; 1 = RLD connected to IN1P
	///
	/// This bit controls the selection of positive inputs from channel 1 for right leg drive derivation.
	bool rld1p : 1;
	// LSB

	uint8_t to_byte() const {
		uint8_t ret = 0;
		ret |= (chop << CHOP_OFFSET);
		ret |= (pdb_rld << PDB_RLD_OFFSET);
		ret |= (rld_loff_sens << RLD_LOFF_SENS_OFFSET);
		ret |= (rld2n << RLD2N_OFFSET);
		ret |= (rld2p << RLD2P_OFFSET);
		ret |= (rld1n << RLD1N_OFFSET);
		ret |= (rld1p << RLD1P_OFFSET);
		return ret;
	}

	static rld_sens_t from_byte(uint8_t byte) {
		rld_sens_t ret;
		ret.chop          = (byte >> CHOP_OFFSET) & 0b11;
		ret.pdb_rld       = (byte >> PDB_RLD_OFFSET) & 1;
		ret.rld_loff_sens = (byte >> RLD_LOFF_SENS_OFFSET) & 1;
		ret.rld2n         = (byte >> RLD2N_OFFSET) & 1;
		ret.rld2p         = (byte >> RLD2P_OFFSET) & 1;
		ret.rld1n         = (byte >> RLD1N_OFFSET) & 1;
		ret.rld1p         = (byte >> RLD1P_OFFSET) & 1;
		return ret;
	}
};

// 导联脱落检测选择寄存器
// Lead-Off Sense Selection
struct loff_sens_t {
	static constexpr uint8_t LOFF_SENS_ADDRESS = 0x07;

	static constexpr uint8_t FLIP2_OFFSET  = 5;
	static constexpr uint8_t FLIP1_OFFSET  = 4;
	static constexpr uint8_t LOFF2N_OFFSET = 3;
	static constexpr uint8_t LOFF2P_OFFSET = 2;
	static constexpr uint8_t LOFF1N_OFFSET = 1;
	static constexpr uint8_t LOFF1P_OFFSET = 0;

	// MSB
	uint8_t must_0 : 2 = 0; // Must be set to '0'
	bool flip2 : 1;         // Current direction selection for channel 2
	bool flip1 : 1;         // Current direction selection for channel 1
	bool loff2n : 1;        // Channel 2 lead-off detection negative inputs
	bool loff2p : 1;        // Channel 2 lead-off detection positive inputs
	bool loff1n : 1;        // Channel 1 lead-off detection negative inputs
	bool loff1p : 1;        // Channel 1 lead-off detection positive inputs
	// LSB

	uint8_t to_byte() const {
		uint8_t ret = 0;
		ret |= (flip2 << FLIP2_OFFSET);
		ret |= (flip1 << FLIP1_OFFSET);
		ret |= (loff2n << LOFF2N_OFFSET);
		ret |= (loff2p << LOFF2P_OFFSET);
		ret |= (loff1n << LOFF1N_OFFSET);
		ret |= (loff1p << LOFF1P_OFFSET);
		// The two most significant bits are reserved and should be 0
		return ret;
	}

	static loff_sens_t from_byte(uint8_t byte) {
		loff_sens_t ret;
		ret.must_0 = 0; // Always set to 0
		ret.flip2  = (byte >> FLIP2_OFFSET) & 1;
		ret.flip1  = (byte >> FLIP1_OFFSET) & 1;
		ret.loff2n = (byte >> LOFF2N_OFFSET) & 1;
		ret.loff2p = (byte >> LOFF2P_OFFSET) & 1;
		ret.loff1n = (byte >> LOFF1N_OFFSET) & 1;
		ret.loff1p = (byte >> LOFF1P_OFFSET) & 1;
		return ret;
	}
};

// 导联脱落检测状态寄存器
// Lead-Off Status
struct PACK loff_stat_t {
	static constexpr uint8_t LOFF_STAT_ADDRESS = 0x08;

	static constexpr uint8_t CLK_DIV_OFFSET  = 6;
	static constexpr uint8_t RLD_STAT_OFFSET = 4;
	static constexpr uint8_t IN2N_OFF_OFFSET = 3;
	static constexpr uint8_t IN2P_OFF_OFFSET = 2;
	static constexpr uint8_t IN1N_OFF_OFFSET = 1;
	static constexpr uint8_t IN1P_OFF_OFFSET = 0;

	static constexpr bool CLK_DIV_512KHZ = 0;
	static constexpr bool CLK_DIV_2MHZ   = 1;

	// MSB

	// Must be set to '0'
	uint8_t must_0_a : 1 = 0;
	/// This bit sets the modultar divider ratio between fCLK and fMOD. Two external clock values are supported: 512 kHz and 2.048 MHz.
	///
	///   - 0 = fMOD = fCLK / 4 (default, use when fCLK = 512 kHz)
	///   - 1 = fMOD = fCLK / 16 (use when fCLK = 2.048 MHz)
	///
	bool clk_div : 1;         // Clock divider selection
	uint8_t must_0_b : 1 = 0; // Must be set to '0'
	bool rld_stat : 1;        // RLD lead-off status (read only)
	bool in2n_off : 1;        // Channel 2 negative electrode status (read only)
	bool in2p_off : 1;        // Channel 2 positive electrode status (read only)
	bool in1n_off : 1;        // Channel 1 negative electrode status (read only)
	bool in1p_off : 1;        // Channel 1 positive electrode status (read only)
	// LSB

	uint8_t to_byte() const {
		uint8_t ret = 0;
		ret |= (clk_div << CLK_DIV_OFFSET);
		// the rest of field is either read only or reserved
		return ret;
	}

	static loff_stat_t from_byte(uint8_t byte) {
		loff_stat_t ret;
		ret.must_0_a = 0; // Always set to 0
		ret.clk_div  = (byte >> CLK_DIV_OFFSET) & 1;
		ret.must_0_b = 0; // Always set to 0
		ret.rld_stat = (byte >> RLD_STAT_OFFSET) & 1;
		ret.in2n_off = (byte >> IN2N_OFF_OFFSET) & 1;
		ret.in2p_off = (byte >> IN2P_OFF_OFFSET) & 1;
		ret.in1n_off = (byte >> IN1N_OFF_OFFSET) & 1;
		ret.in1p_off = (byte >> IN1P_OFF_OFFSET) & 1;
		return ret;
	}
};

// 呼吸检测控制寄存器 1
struct PACK resp1_t {
	static constexpr uint8_t RESP1_ADDRESS = 0x09;

	static constexpr uint8_t RESP_DEMOD_EN1_OFFSET = 7;
	static constexpr uint8_t RESP_MOD_EN_OFFSET    = 6;
	static constexpr uint8_t RESP_PH_OFFSET        = 2;
	static constexpr uint8_t RESP_CTRL_OFFSET      = 0;

	// RESP_PH settings (32 kHz)
	static constexpr uint8_t RESP_PH_32k_0      = 0b0000;
	static constexpr uint8_t RESP_PH_32k_11_25  = 0b0001;
	static constexpr uint8_t RESP_PH_32k_22_5   = 0b0010;
	static constexpr uint8_t RESP_PH_32k_33_75  = 0b0011;
	static constexpr uint8_t RESP_PH_32k_45     = 0b0100;
	static constexpr uint8_t RESP_PH_32k_56_25  = 0b0101;
	static constexpr uint8_t RESP_PH_32k_67_5   = 0b0110;
	static constexpr uint8_t RESP_PH_32k_78_75  = 0b0111;
	static constexpr uint8_t RESP_PH_32k_90     = 0b1000;
	static constexpr uint8_t RESP_PH_32k_101_25 = 0b1001;
	static constexpr uint8_t RESP_PH_32k_112_5  = 0b1010;
	static constexpr uint8_t RESP_PH_32k_123_75 = 0b1011;
	static constexpr uint8_t RESP_PH_32k_135    = 0b1100;
	static constexpr uint8_t RESP_PH_32k_146_25 = 0b1101;
	static constexpr uint8_t RESP_PH_32k_157_5  = 0b1110;
	static constexpr uint8_t RESP_PH_32k_168_75 = 0b1111;
	// RESP_PH settings (64 kHz)
	static constexpr uint8_t RESP_PH_64k_0     = 0b0000;
	static constexpr uint8_t RESP_PH_64k_22_5  = 0b0001;
	static constexpr uint8_t RESP_PH_64k_45    = 0b0010;
	static constexpr uint8_t RESP_PH_64k_67_5  = 0b0011;
	static constexpr uint8_t RESP_PH_64k_90    = 0b0100;
	static constexpr uint8_t RESP_PH_64k_112_5 = 0b0101;
	static constexpr uint8_t RESP_PH_64k_135   = 0b0110;
	static constexpr uint8_t RESP_PH_64k_157_5 = 0b0111;
	// the rest of the values are not available for 64 kHz

	// MSB

	/// Enables respiration demodulation circuitry
	bool resp_demod_en1 : 1;
	bool resp_mod_en : 1;   // Enables respiration modulation circuitry
	uint8_t resp_ph : 4;    // Respiration phase
	uint8_t must_1 : 1 = 1; // Must be set to '1'
	/// Respiration control
	/// This bit sets the mode of the respiration circuitry.
	///   - 0 = Internal respiration with internal clock
	///   - 1 = Internal respiration with external clock
	bool resp_ctrl : 1;
	// LSB

	uint8_t to_byte() const {
		uint8_t ret = 0;
		ret |= (resp_demod_en1 << RESP_DEMOD_EN1_OFFSET);
		ret |= (resp_mod_en << RESP_MOD_EN_OFFSET);
		ret |= (resp_ph << RESP_PH_OFFSET);
		ret |= (1 << 1); // Reserved bit must be set to '1'
		ret |= resp_ctrl;
		return ret;
	}

	static resp1_t from_byte(uint8_t byte) {
		resp1_t ret;
		ret.resp_demod_en1 = (byte >> RESP_DEMOD_EN1_OFFSET) & 1;
		ret.resp_mod_en    = (byte >> RESP_MOD_EN_OFFSET) & 1;
		ret.resp_ph        = (byte >> RESP_PH_OFFSET) & 0xF;
		ret.must_1         = 1; // Always set to 1
		ret.resp_ctrl      = byte & 1;
		return ret;
	}
};

// 呼吸检测控制寄存器 2
struct PACK resp2_t {
	static constexpr uint8_t RESP2_ADDRESS = 0x0A;

	static constexpr uint8_t CALIB_ON_OFFSET   = 7;
	static constexpr uint8_t RESP_FREQ_OFFSET  = 2;
	static constexpr uint8_t RLDREF_INT_OFFSET = 1;

	static constexpr bool RESP_FREQ_32K = false; // Default
	static constexpr bool RESP_FREQ_64K = true;

	// 外部馈电
	static constexpr bool RLDREF_INT_EXTERN = false;
	// 内部馈电 (default)
	static constexpr bool RLDREF_INT_INTERNAL = true;

	// MSB

	/// Calibration on
	///
	/// This bit is used to enable offset calibration.
	///   - 0 = Off
	///   - 1 = On
	bool calib_on : 1;
	uint8_t must_0 : 4 = 0;
	/// Respiration control frequency (ADS1292R only)
	///
	/// This bit controls the respiration control frequency when RESP_CTRL = 0.
	/// This bit must be written with '1' for the ADS1291 and ADS1292.
	bool resp_freq : 1;
	/// RLDREF signal
	///
	/// This bit determines the RLDREF signal source.
	///
	///   - 0 = RLDREF signal fed externally
	///   - 1 = RLDREF signal (AVDD – AVSS) / 2 generated internally (default)
	bool rldref_int : 1;
	uint8_t must_1 : 1 = 1;
	// LSB

	uint8_t to_byte() const {
		uint8_t ret = 0;
		ret |= (calib_on << CALIB_ON_OFFSET);
		ret |= (resp_freq << RESP_FREQ_OFFSET);
		ret |= (rldref_int << RLDREF_INT_OFFSET);
		ret |= (1 << 0); // Reserved2 bit must be set to '1'
		return ret;
	}

	static resp2_t from_byte(uint8_t byte) {
		resp2_t ret;
		ret.calib_on   = (byte >> CALIB_ON_OFFSET) & 1;
		ret.must_0     = 0; // Always set to 0
		ret.resp_freq  = (byte >> RESP_FREQ_OFFSET) & 1;
		ret.rldref_int = (byte >> RLDREF_INT_OFFSET) & 1;
		ret.must_1     = 1; // Always set to 1
		return ret;
	}
};

struct PACK gpio_t {
	static constexpr uint8_t GPIO_ADDRESS = 0x0B;

	static constexpr uint8_t GPIOC_OFFSET = 2;
	static constexpr uint8_t GPIOD_OFFSET = 0;

	// MSB

	uint8_t must_0 : 4 = 0;
	/// GPIO2 control
	/// These bits determine if the corresponding GPIOD pin is an input or output.
	///   - 0 = Input
	///   - 1 = Output
	bool gpioc2:1;
	/// GPIO1 control
	bool gpioc1:1;
	/// GPIO2 data
	///
	/// These bits are used to read and write data to the GPIO ports.
	///
	/// When reading the register, the data returned correspond to the state of the GPIO external pins,
	/// whether they are programmed as inputs or as outputs.
	///
	/// As outputs, a write to the GPIOD sets the output value.
	/// As inputs, a write to the GPIOD has no effect.
	///
	/// GPIO is not available in certain respiration modes.
	bool gpiod2:1;
	/// GPIO1 data
	bool gpiod1:1;
	// LSB

	uint8_t to_byte() const {
		uint8_t ret = 0;
		// Reserved bits are already 0
		ret |= (gpioc2 << (GPIOC_OFFSET + 1));
		ret |= (gpioc1 << GPIOC_OFFSET);
		ret |= (gpiod2 << (GPIOD_OFFSET + 1));
		ret |= (gpiod1 << GPIOD_OFFSET);
		return ret;
	}

	static gpio_t from_byte(uint8_t byte) {
		gpio_t ret;
		ret.must_0 = 0;  // Always set to 0
		ret.gpioc2 = (byte >> (GPIOC_OFFSET + 1)) & 1;
		ret.gpioc1 = (byte >> GPIOC_OFFSET) & 1;
		ret.gpiod2 = (byte >> (GPIOD_OFFSET + 1)) & 1;
		ret.gpiod1 = (byte >> GPIOD_OFFSET) & 1;
		return ret;
	}
};

constexpr auto DATA_BYTES = 9;
constexpr auto DATA_BITS  = DATA_BYTES * 8;

#undef PACK
} // namespace ADS1292R

#endif //ADS1292R_H
