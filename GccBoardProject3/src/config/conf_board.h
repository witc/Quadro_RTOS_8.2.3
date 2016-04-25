/**
 * \file
 *
 * \brief User board configuration template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H


//Semtech - RF
#define SX1276_CS_PIN			PIO_PA11_IDX // OUT
#define SX1276_SCK_PIN			PIO_PA14_IDX // OUT
#define SX1276_MOSI_PIN			PIO_PA13_IDX // OUT
#define SX1276_MISO_PIN			PIO_PA12_IDX // OUT
#define SX1276_RESET_PIN		PIO_PA24_IDX // OUT
#define SX1276_RxTx_PIN			PIO_PA6_IDX // OUT
#define SX1276_NIRQ0_PIN		PIO_PA18_IDX // IN
#define SX1276_NIRQ1_PIN		PIO_PA17_IDX // IN


//ADNS - 3080
#define ADNS_CS_PIN			PIO_PA10_IDX // OUT
#define ADNS_RESET_PIN		PIO_PC25_IDX // OUT

/*  MPU */
#define I2C_SCL_PIN				PIO_PA4_IDX
#define I2C_SDA_PIN				PIO_PA3_IDX
#define EXT1_TWI_SDA_MUX			1
#define EXT1_TWI_SCL_MUX			1
#define MPU9150_CS_PIN 			PIO_PA9_IDX
#define MPU9150_PIN_INT			PIO_PB0_IDX
#define TWI0_DATA_FLAGS  (PIO_PERIPH_A | PIO_PULLUP)
#define TWI0_CLK_GPIO    PIO_PA4_IDX
#define TWI0_CLK_FLAGS   (PIO_PERIPH_A | PIO_PULLUP)

//Measure Periode
#define PERIODE_PIN				PIO_PC24_IDX
#define PERIODE_PIN_INT			PIO_PC27_IDX

/* PWM motors*/
#define PWM_M1					PIO_PA23_IDX //PER_B	PWM_H0
#define PWM_M2					PIO_PA1_IDX  //PER_A	PWM_H1
#define PWM_M3					PIO_PA25_IDX //PER_B	PWM_H2
#define PWM_M4					PIO_PC21_IDX //PER_B	PWM_H3


/* LED */
#define LEDW					PIO_PB1_IDX
#define LEDR					PIO_PB5_IDX
#define LED0					PIO_PC23_IDX
//#define LEDG					PIO_PB2_IDX

//GPS
#define GPS_RX_PIN			    PIO_PA21_IDX
#define GPS_TX_PIN		        PIO_PA22_IDX
#define GPS_RESET_PIN		    PIO_PC26_IDX
#define GPS_USART				USART1

/** SPI MISO pin definition. */
#define SPI_MISO_GPIO         (PIO_PA12_IDX)
#define SPI_MISO_FLAGS       (PIO_PERIPH_A | PIO_PULLUP)
/** SPI MOSI pin definition. */
#define SPI_MOSI_GPIO         (PIO_PA13_IDX)
#define SPI_MOSI_FLAGS       (PIO_PERIPH_A | PIO_PULLUP)
/** SPI SPCK pin definition. */
#define SPI_SPCK_GPIO         (PIO_PA14_IDX)
#define SPI_SPCK_FLAGS       (PIO_PERIPH_A | PIO_PULLUP)


/** SPI chip select 0 pin definition. (Only one configuration is possible) */
#define SPI_NPCS0_GPIO         (PIO_PA11_IDX)
#define SPI_NPCS0_FLAGS           (PIO_PERIPH_A | PIO_DEFAULT)
/** SPI chip select 1 pin definition. (multiple configurations are possible) */
#define SPI_NPCS1_PA9_GPIO     (PIO_PA9_IDX)
#define SPI_NPCS1_PA9_FLAGS       (PIO_PERIPH_B | PIO_DEFAULT)
#define SPI_NPCS1_PA31_GPIO    (PIO_PA31_IDX)
#define SPI_NPCS1_PA31_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)
#define SPI_NPCS1_PB14_GPIO    (PIO_PB14_IDX)
#define SPI_NPCS1_PB14_FLAGS      (PIO_PERIPH_A | PIO_DEFAULT)
#define SPI_NPCS1_PC4_GPIO     (PIO_PC4_IDX)
#define SPI_NPCS1_PC4_FLAGS       (PIO_PERIPH_B | PIO_DEFAULT)
/** SPI chip select 2 pin definition. (multiple configurations are possible) */
#define SPI_NPCS2_PA10_GPIO    (PIO_PA10_IDX)
#define SPI_NPCS2_PA10_FLAGS      (PIO_PERIPH_B | PIO_PULLUP)
#define SPI_NPCS2_PA30_GPIO    (PIO_PA30_IDX)
#define SPI_NPCS2_PA30_FLAGS      (PIO_PERIPH_B | PIO_DEFAULT)
#define SPI_NPCS2_PB2_GPIO     (PIO_PB2_IDX)
#define SPI_NPCS2_PB2_FLAGS       (PIO_PERIPH_B | PIO_DEFAULT)
/** SPI chip select 3 pin definition. (multiple configurations are possible) */
#define SPI_NPCS3_PA3_GPIO     (PIO_PA3_IDX)
#define SPI_NPCS3_PA3_FLAGS       (PIO_PERIPH_B | PIO_DEFAULT)
#define SPI_NPCS3_PA5_GPIO     (PIO_PA5_IDX)
#define SPI_NPCS3_PA5_FLAGS       (PIO_PERIPH_B | PIO_DEFAULT)
#define SPI_NPCS3_PA22_GPIO    (PIO_PA22_IDX)
#define SPI_NPCS3_PA22_FLAGS      (PIO_PERIPH_B | PIO_DEFAULT)
//@}

#endif // CONF_BOARD_H
