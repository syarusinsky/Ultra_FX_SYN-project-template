#include "../../lib/STM32h745zi-HAL/llpd/include/LLPD.hpp"

#include "EEPROM_CAT24C64.hpp"
#include "SRAM_23K256.hpp"
#include "SDCard.hpp"
#include "OLED_SH1106.hpp"

#define SYS_CLOCK_FREQUENCY = 480000000;

// global variables
volatile bool adcSetupComplete = false; // should be set to true after adc has been initialized
constexpr unsigned int numSquareWaveSamples = 80;
volatile unsigned int squareWaveCurrentSampleNum = 0;
volatile uint16_t squareWaveBuffer[numSquareWaveSamples];

// peripheral defines
#define OP_AMP1_INV_OUT_PORT 		GPIO_PORT::C
#define OP_AMP1_NONINVR_PORT 		GPIO_PORT::B
#define OP_AMP1_INVERT_PIN 		GPIO_PIN::PIN_5
#define OP_AMP1_OUTPUT_PIN 		GPIO_PIN::PIN_4
#define OP_AMP1_NON_INVERT_PIN 		GPIO_PIN::PIN_0
#define OP_AMP2_PORT 			GPIO_PORT::E
#define OP_AMP2_INVERT_PIN 		GPIO_PIN::PIN_8
#define OP_AMP2_OUTPUT_PIN 		GPIO_PIN::PIN_7
#define OP_AMP2_NON_INVERT_PIN 		GPIO_PIN::PIN_9
#define EFFECT_ADC_PORT 		GPIO_PORT::A
#define EFFECT_ADC_NUM 			ADC_NUM::ADC_1_2
#define EFFECT1_ADC_PIN 		GPIO_PIN::PIN_2
#define EFFECT1_ADC_CHANNEL 		ADC_CHANNEL::CHAN_14
#define EFFECT2_ADC_PIN 		GPIO_PIN::PIN_3
#define EFFECT2_ADC_CHANNEL 		ADC_CHANNEL::CHAN_15
#define EFFECT3_ADC_PIN 		GPIO_PIN::PIN_0
#define EFFECT3_ADC_CHANNEL 		ADC_CHANNEL::CHAN_16
#define AUDIO_IN_PORT 			GPIO_PORT::C
#define AUDIO_IN_ADC_NUM 		ADC_NUM::ADC_3
#define AUDIO1_IN_PIN 			GPIO_PIN::PIN_2
#define AUDIO1_IN_ADC_CHANNEL 		ADC_CHANNEL::CHAN_0
#define AUDIO2_IN_PIN 			GPIO_PIN::PIN_3
#define AUDIO2_IN_ADC_CHANNEL 		ADC_CHANNEL::CHAN_1
#define EFFECT_BUTTON_PORT 		GPIO_PORT::E
#define EFFECT1_BUTTON_PIN 		GPIO_PIN::PIN_0
#define EFFECT2_BUTTON_PIN 		GPIO_PIN::PIN_1
#define SRAM_CS_PORT 			GPIO_PORT::D
#define SRAM1_CS_PIN 			GPIO_PIN::PIN_8
#define SRAM2_CS_PIN 			GPIO_PIN::PIN_9
#define SRAM3_CS_PIN 			GPIO_PIN::PIN_10
#define SRAM4_CS_PIN 			GPIO_PIN::PIN_11
#define EEPROM1_ADDRESS 		false, false, false
#define EEPROM2_ADDRESS 		true, false, false
#define EEPROM3_ADDRESS 		false, true, false
#define EEPROM4_ADDRESS 		true, true, false
#define SD_CARD_CS_PORT 		GPIO_PORT::E
#define SD_CARD_CS_PIN 			GPIO_PIN::PIN_15
#define OLED_PORT 			GPIO_PORT::C
#define OLED_RESET_PIN 			GPIO_PIN::PIN_13
#define OLED_DC_PIN 			GPIO_PIN::PIN_14
#define OLED_CS_PIN 			GPIO_PIN::PIN_11
#define MIDI_USART_NUM 			USART_NUM::USART_6
#define LOGGING_USART_NUM 		USART_NUM::USART_2
#define EEPROM_I2C_NUM 			I2C_NUM::I2C_1
#define SRAM_SPI_NUM 			SPI_NUM::SPI_2
#define SD_CARD_SPI_NUM 		SPI_NUM::SPI_4
#define OLED_SPI_NUM 			SPI_NUM::SPI_3

// these pins are unconnected on Ultra_FX_SYN Rev 2 development board, so we disable them as per the ST recommendations
void disableUnusedPIns()
{
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_1, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_7, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_8, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_9, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_10, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_11, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_12, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::A, GPIO_PIN::PIN_15, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );

	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_1, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_2, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_4, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_5, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_8, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_9, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_10, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_11, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::B, GPIO_PIN::PIN_12, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );

	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_0, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_1, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_8, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_9, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::C, GPIO_PIN::PIN_15, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );

	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_0, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_1, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_2, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_3, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_4, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_5, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_6, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_7, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_12, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_13, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_14, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::D, GPIO_PIN::PIN_15, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );

	LLPD::gpio_output_setup( GPIO_PORT::E, GPIO_PIN::PIN_2, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::E, GPIO_PIN::PIN_3, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::E, GPIO_PIN::PIN_4, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::E, GPIO_PIN::PIN_5, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::E, GPIO_PIN::PIN_6, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::E, GPIO_PIN::PIN_10, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::E, GPIO_PIN::PIN_11, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );

	LLPD::gpio_output_setup( GPIO_PORT::F, GPIO_PIN::PIN_6, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::F, GPIO_PIN::PIN_8, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::F, GPIO_PIN::PIN_10, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::F, GPIO_PIN::PIN_11, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::F, GPIO_PIN::PIN_14, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::F, GPIO_PIN::PIN_15, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );

	LLPD::gpio_output_setup( GPIO_PORT::G, GPIO_PIN::PIN_6, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::G, GPIO_PIN::PIN_7, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::G, GPIO_PIN::PIN_8, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::G, GPIO_PIN::PIN_9, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::G, GPIO_PIN::PIN_10, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::G, GPIO_PIN::PIN_11, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::G, GPIO_PIN::PIN_12, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::G, GPIO_PIN::PIN_13, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
	LLPD::gpio_output_setup( GPIO_PORT::G, GPIO_PIN::PIN_14, GPIO_PUPD::PULL_DOWN, GPIO_OUTPUT_TYPE::PUSH_PULL,
					GPIO_OUTPUT_SPEED::LOW );
}

int main(void)
{
	// TODO maybe move this stuff as well as caching to the HAL?
	// disable mpu
	__DMB();
	SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;
	MPU->CTRL = 0;
	// region config for sram4 as non-cacheable
	MPU->RNR = 0;
	MPU->RBAR = D3_SRAM_BASE;
	MPU->RASR = 	0 			 	<< 	MPU_RASR_XN_Pos 	|
			ARM_MPU_AP_FULL 	 	<< 	MPU_RASR_AP_Pos 	|
			0 			 	<< 	MPU_RASR_TEX_Pos 	|
			0 			 	<< 	MPU_RASR_S_Pos 		|
			0 			 	<< 	MPU_RASR_C_Pos 		|
			0 			 	<< 	MPU_RASR_B_Pos 		|
			0 			 	<< 	MPU_RASR_SRD_Pos 	|
			ARM_MPU_REGION_SIZE_64KB 	<< 	MPU_RASR_SIZE_Pos 	|
			1 				<< 	MPU_RASR_ENABLE_Pos;
	// enable mpu
	MPU->CTRL = MPU_CTRL_PRIVDEFENA_Msk | MPU_CTRL_ENABLE_Msk;
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
	__DSB();
	__ISB();

	// setup clock 480MHz (also prescales peripheral clocks to fit rate limitations)
	LLPD::rcc_clock_start_max_cpu1();

	// enable gpio clocks
	LLPD::gpio_enable_clock( GPIO_PORT::A );
	LLPD::gpio_enable_clock( GPIO_PORT::B );
	LLPD::gpio_enable_clock( GPIO_PORT::C );
	LLPD::gpio_enable_clock( GPIO_PORT::D );
	LLPD::gpio_enable_clock( GPIO_PORT::E );
	LLPD::gpio_enable_clock( GPIO_PORT::F );
	LLPD::gpio_enable_clock( GPIO_PORT::G );
	LLPD::gpio_enable_clock( GPIO_PORT::H );

	// USART setup
	LLPD::usart_init( LOGGING_USART_NUM, USART_WORD_LENGTH::BITS_8, USART_PARITY::NONE, USART_CONF::TX_AND_RX,
				USART_STOP_BITS::BITS_1, 120000000, 9600 );
	LLPD::usart_log( LOGGING_USART_NUM, "Ultra_FX_SYN starting up ----------------------------" );

	// audio timer setup (for 40 kHz sampling rate at 480 MHz / 2 timer clock)
	LLPD::tim6_counter_setup( 0, 6000, 30000 );
	LLPD::tim6_counter_enable_interrupts();
	LLPD::usart_log( LOGGING_USART_NUM, "tim6 initialized..." );

	// DAC setup
	LLPD::dac_init_use_dma( true );
	LLPD::usart_log( LOGGING_USART_NUM, "dac initialized..." );

	// Op Amp setup
	LLPD::gpio_analog_setup( OP_AMP1_INV_OUT_PORT, OP_AMP1_INVERT_PIN );
	LLPD::gpio_analog_setup( OP_AMP1_INV_OUT_PORT, OP_AMP1_OUTPUT_PIN );
	LLPD::gpio_analog_setup( OP_AMP1_NONINVR_PORT, OP_AMP1_NON_INVERT_PIN );
	LLPD::opamp_init( OPAMP_NUM::OPAMP_1 );
	LLPD::gpio_analog_setup( OP_AMP2_PORT, OP_AMP2_INVERT_PIN );
	LLPD::gpio_analog_setup( OP_AMP2_PORT, OP_AMP2_OUTPUT_PIN );
	LLPD::gpio_analog_setup( OP_AMP2_PORT, OP_AMP2_NON_INVERT_PIN );
	LLPD::opamp_init( OPAMP_NUM::OPAMP_2 );
	LLPD::usart_log( LOGGING_USART_NUM, "op amp initialized..." );

	// spi initialization
	LLPD::spi_master_init( SRAM_SPI_NUM, SPI_BAUD_RATE::SYSCLK_DIV_BY_256, SPI_CLK_POL::LOW_IDLE, SPI_CLK_PHASE::FIRST,
				SPI_DUPLEX::FULL, SPI_FRAME_FORMAT::MSB_FIRST, SPI_DATA_SIZE::BITS_8 );
	LLPD::spi_master_init( SD_CARD_SPI_NUM, SPI_BAUD_RATE::SYSCLK_DIV_BY_256, SPI_CLK_POL::LOW_IDLE, SPI_CLK_PHASE::FIRST,
				SPI_DUPLEX::FULL, SPI_FRAME_FORMAT::MSB_FIRST, SPI_DATA_SIZE::BITS_8 );
	LLPD::spi_master_init( OLED_SPI_NUM, SPI_BAUD_RATE::SYSCLK_DIV_BY_256, SPI_CLK_POL::LOW_IDLE, SPI_CLK_PHASE::FIRST,
				SPI_DUPLEX::FULL, SPI_FRAME_FORMAT::MSB_FIRST, SPI_DATA_SIZE::BITS_8 );
	LLPD::usart_log( LOGGING_USART_NUM, "spi initialized..." );

	// i2c initialization
	LLPD::i2c_master_setup( EEPROM_I2C_NUM, 0x308075AE );
	LLPD::usart_log( LOGGING_USART_NUM, "i2c initialized..." );

	// audio timer start
	LLPD::tim6_counter_start();
	LLPD::usart_log( LOGGING_USART_NUM, "tim6 started..." );

	// adc setup (note this must be done after the tim6_counter_start() call since it uses the delay funtion)
	LLPD::gpio_analog_setup( EFFECT_ADC_PORT, EFFECT1_ADC_PIN );
	LLPD::gpio_analog_setup( EFFECT_ADC_PORT, EFFECT2_ADC_PIN );
	LLPD::gpio_analog_setup( EFFECT_ADC_PORT, EFFECT3_ADC_PIN );
	LLPD::gpio_analog_setup( AUDIO_IN_PORT, AUDIO1_IN_PIN );
	LLPD::gpio_analog_setup( AUDIO_IN_PORT, AUDIO2_IN_PIN );
	LLPD::adc_init( ADC_NUM::ADC_1_2, ADC_CYCLES_PER_SAMPLE::CPS_64p5 );
	LLPD::adc_init( ADC_NUM::ADC_3, ADC_CYCLES_PER_SAMPLE::CPS_32p5 );
	LLPD::adc_set_channel_order( ADC_NUM::ADC_1_2, 3, EFFECT1_ADC_CHANNEL, EFFECT2_ADC_CHANNEL, EFFECT3_ADC_CHANNEL );
	LLPD::adc_set_channel_order( ADC_NUM::ADC_3, 2, AUDIO1_IN_ADC_CHANNEL, AUDIO2_IN_ADC_CHANNEL );
	adcSetupComplete = true;

	// pushbutton setup
	LLPD::gpio_digital_input_setup( EFFECT_BUTTON_PORT, EFFECT1_BUTTON_PIN, GPIO_PUPD::PULL_UP );
	LLPD::gpio_digital_input_setup( EFFECT_BUTTON_PORT, EFFECT2_BUTTON_PIN, GPIO_PUPD::PULL_UP );

	// SD Card setup and test
	LLPD::gpio_output_setup( SD_CARD_CS_PORT, SD_CARD_CS_PIN, GPIO_PUPD::PULL_UP, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::HIGH, false );
	LLPD::gpio_output_set( SD_CARD_CS_PORT, SD_CARD_CS_PIN, true );
	SDCard sdCard( SD_CARD_SPI_NUM, SD_CARD_CS_PORT, SD_CARD_CS_PIN );
	sdCard.initialize();
	LLPD::usart_log( LOGGING_USART_NUM, "sd card initialized..." );
	// TODO comment the verification lines out if you're using the sd card for persistent memory
	SharedData<uint8_t> sdCardValsToWrite = SharedData<uint8_t>::MakeSharedData( 3 );
	sdCardValsToWrite[0] = 23; sdCardValsToWrite[1] = 87; sdCardValsToWrite[2] = 132;
	sdCard.writeToMedia( sdCardValsToWrite, 54 );
	SharedData<uint8_t> retVals3 = sdCard.readFromMedia( 3, 54 );
	if ( retVals3[0] == 23 && retVals3[1] == 87 && retVals3[2] == 132 )
	{
		LLPD::usart_log( USART_NUM::USART_3, "sd card verified..." );
	}
	else
	{
		LLPD::usart_log( USART_NUM::USART_3, "WARNING!!! sd card failed verification..." );
	}

	// SRAM setup and test
	std::vector<Sram_23K256_GPIO_Config> spiGpioConfigs;
	spiGpioConfigs.emplace_back( SRAM_CS_PORT, SRAM1_CS_PIN );
	spiGpioConfigs.emplace_back( SRAM_CS_PORT, SRAM2_CS_PIN );
	spiGpioConfigs.emplace_back( SRAM_CS_PORT, SRAM3_CS_PIN );
	spiGpioConfigs.emplace_back( SRAM_CS_PORT, SRAM4_CS_PIN );
	Sram_23K256_Manager srams( SRAM_SPI_NUM, spiGpioConfigs );
	SharedData<uint8_t> sramValsToWrite = SharedData<uint8_t>::MakeSharedData( 3 );
	sramValsToWrite[0] = 25; sramValsToWrite[1] = 16; sramValsToWrite[2] = 8;
	srams.writeToMedia( sramValsToWrite, 45 );
	srams.writeToMedia( sramValsToWrite, 45 + Sram_23K256::SRAM_SIZE );
	srams.writeToMedia( sramValsToWrite, 45 + Sram_23K256::SRAM_SIZE * 2 );
	srams.writeToMedia( sramValsToWrite, 45 + Sram_23K256::SRAM_SIZE * 3 );
	SharedData<uint8_t> sram1Verification = srams.readFromMedia( 3, 45 );
	SharedData<uint8_t> sram2Verification = srams.readFromMedia( 3, 45 + Sram_23K256::SRAM_SIZE );
	SharedData<uint8_t> sram3Verification = srams.readFromMedia( 3, 45 + Sram_23K256::SRAM_SIZE * 2 );
	SharedData<uint8_t> sram4Verification = srams.readFromMedia( 3, 45 + Sram_23K256::SRAM_SIZE * 3 );
	if ( sram1Verification[0] == 25 && sram1Verification[1] == 16 && sram1Verification[2] == 8 &&
			sram2Verification[0] == 25 && sram2Verification[1] == 16 && sram2Verification[2] == 8 &&
			sram3Verification[0] == 25 && sram3Verification[1] == 16 && sram3Verification[2] == 8 &&
			sram4Verification[0] == 25 && sram4Verification[1] == 16 && sram4Verification[2] == 8 )
	{
		LLPD::usart_log( LOGGING_USART_NUM, "srams verified..." );
	}
	else
	{
		LLPD::usart_log( LOGGING_USART_NUM, "WARNING!!! srams failed verification..." );
	}

	// EEPROM setup and test
	std::vector<Eeprom_CAT24C64_AddressConfig> eepromAddressConfigs;
	eepromAddressConfigs.emplace_back( EEPROM1_ADDRESS );
	eepromAddressConfigs.emplace_back( EEPROM2_ADDRESS );
	eepromAddressConfigs.emplace_back( EEPROM3_ADDRESS );
	eepromAddressConfigs.emplace_back( EEPROM4_ADDRESS );
	Eeprom_CAT24C64_Manager eeproms( EEPROM_I2C_NUM, eepromAddressConfigs );
	// TODO comment the verification lines out if you're using the eeprom for persistent memory
	SharedData<uint8_t> eepromValsToWrite = SharedData<uint8_t>::MakeSharedData( 3 );
	eepromValsToWrite[0] = 64; eepromValsToWrite[1] = 23; eepromValsToWrite[2] = 17;
	eeproms.writeToMedia( eepromValsToWrite, 45 );
	eeproms.writeToMedia( eepromValsToWrite, 45 + Eeprom_CAT24C64::EEPROM_SIZE );
	SharedData<uint8_t> eeprom1Verification = eeproms.readFromMedia( 3, 45 );
	SharedData<uint8_t> eeprom2Verification = eeproms.readFromMedia( 3, 45 + Eeprom_CAT24C64::EEPROM_SIZE );
	if ( eeprom1Verification[0] == 64 && eeprom1Verification[1] == 23 && eeprom1Verification[2] == 17 &&
			eeprom2Verification[0] == 64 && eeprom2Verification[1] == 23 && eeprom2Verification[2] == 17 )
	{
		LLPD::usart_log( LOGGING_USART_NUM, "eeproms verified..." );
	}
	else
	{
		LLPD::usart_log( LOGGING_USART_NUM, "WARNING!!! eeproms failed verification..." );
	}

	// display buffer
	uint8_t displayBuffer[(SH1106_LCDWIDTH * SH1106_LCDHEIGHT) / 8] = { 0 };

	// fill display buffer
	for ( unsigned int byte = 0; byte < (SH1106_LCDHEIGHT * SH1106_LCDWIDTH) / 8; byte++ )
	{
		displayBuffer[byte] = 0xFF;
	}

	// OLED setup
	LLPD::gpio_output_setup( OLED_PORT, OLED_CS_PIN, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::HIGH, false );
	LLPD::gpio_output_set( OLED_PORT, OLED_CS_PIN, true );
	LLPD::gpio_output_setup( OLED_PORT, OLED_DC_PIN, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::HIGH, false );
	LLPD::gpio_output_set( OLED_PORT, OLED_DC_PIN, true );
	LLPD::gpio_output_setup( OLED_PORT, OLED_RESET_PIN, GPIO_PUPD::NONE, GPIO_OUTPUT_TYPE::PUSH_PULL, GPIO_OUTPUT_SPEED::HIGH, false );
	LLPD::gpio_output_set( OLED_PORT, OLED_RESET_PIN, true );
	Oled_SH1106 oled( OLED_SPI_NUM, OLED_PORT, OLED_CS_PIN, OLED_PORT, OLED_DC_PIN, OLED_PORT, OLED_RESET_PIN );
	oled.begin();
	oled.displayFullRowMajor( displayBuffer );
	LLPD::usart_log( LOGGING_USART_NUM, "oled initialized..." );

	LLPD::usart_log( LOGGING_USART_NUM, "Ultra_FX_SYN setup complete, entering while loop -------------------------------" );

	// create audio buffer of 1KHz square wave
	for ( unsigned int sampleNum = 0; sampleNum < numSquareWaveSamples; sampleNum++ )
	{
		if ( sampleNum < numSquareWaveSamples / 2 )
		{
			squareWaveBuffer[sampleNum] = 4095;
		}
		else
		{
			squareWaveBuffer[sampleNum] = 0;
		}
	}

	// flush denormals
	__set_FPSCR( __get_FPSCR() | (1 << 24) );

	// enable instruction cache
	SCB_EnableICache();

	// enable data cache (will only be useful for constant values stored in flash)
	SCB_InvalidateDCache();
	SCB_EnableDCache();

	while ( true )
	{
		LLPD::adc_perform_conversion_sequence( EFFECT_ADC_NUM );
		uint16_t effect1Val = LLPD::adc_get_channel_value( EFFECT_ADC_NUM, EFFECT1_ADC_CHANNEL );
		uint16_t effect2Val = LLPD::adc_get_channel_value( EFFECT_ADC_NUM, EFFECT2_ADC_CHANNEL );
		uint16_t effect3Val = LLPD::adc_get_channel_value( EFFECT_ADC_NUM, EFFECT3_ADC_CHANNEL );
		LLPD::usart_log_int( LOGGING_USART_NUM, "EFFECT 1 VALUE: ", effect1Val );
		LLPD::usart_log_int( LOGGING_USART_NUM, "EFFECT 2 VALUE: ", effect2Val );
		LLPD::usart_log_int( LOGGING_USART_NUM, "EFFECT 3 VALUE: ", effect3Val );

		if ( ! LLPD::gpio_input_get(EFFECT_BUTTON_PORT, EFFECT1_BUTTON_PIN) )
		{
			LLPD::usart_log( LOGGING_USART_NUM, "BUTTON 1 PRESSED" );
		}

		if ( ! LLPD::gpio_input_get(EFFECT_BUTTON_PORT, EFFECT2_BUTTON_PIN) )
		{
			LLPD::usart_log( LOGGING_USART_NUM, "BUTTON 2 PRESSED" );
		}
	}
}

extern "C" void TIM6_DAC_IRQHandler (void)
{
	if ( ! LLPD::tim6_isr_handle_delay() ) // if not currently in a delay function,...
	{
		// LLPD::dac_send( squareWaveBuffer[squareWaveCurrentSampleNum], squareWaveBuffer[squareWaveCurrentSampleNum] );
		// squareWaveCurrentSampleNum = ( squareWaveCurrentSampleNum + 1 ) % numSquareWaveSamples;

		// if ( adcSetupComplete )
		// {
		// 	LLPD::adc_perform_conversion_sequence( AUDIO_IN_ADC_NUM );
		// 	uint16_t audioIn1 = LLPD::adc_get_channel_value( AUDIO_IN_ADC_NUM, AUDIO1_IN_ADC_CHANNEL );
		// 	uint16_t audioIn2 = LLPD::adc_get_channel_value( AUDIO_IN_ADC_NUM, AUDIO2_IN_ADC_CHANNEL );
		// 	LLPD::dac_send( audioIn1, audioIn2 );
		// }
	}

	LLPD::tim6_counter_clear_interrupt_flag();
}

extern "C" void USART2_IRQHandler (void) // logging usart
{
	// loopback test code for usart recieve
	uint16_t data = LLPD::usart_receive( LOGGING_USART_NUM );
	LLPD::usart_transmit( LOGGING_USART_NUM, data );
}
