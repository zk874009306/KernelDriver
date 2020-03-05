/*
 * Copyright 2017, NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v3.0
processor: LPC54018
package_id: LPC54018JET180
mcu_data: ksdk2_0
processor_version: 0.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iocon.h"
#include "pin_mux.h"
#include "LPC54018.h"

/*FUNCTION**********************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 *END**************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();
    SPI9_InitPins();
}

#define IOCON_PIO_DIGITAL_EN        0x0100u   /*!< Enables digital function */
#define IOCON_PIO_FUNC1               0x01u   /*!< Selects pin function 1 */
#define IOCON_PIO_FUNC5               0x05u   /*!< Selects pin function 5 */
#define IOCON_PIO_FUNC7               0x07u   /*!< Selects pin function 7 */
#define IOCON_PIO_INPFILT_OFF       0x0200u   /*!< Input filter disabled */
#define IOCON_PIO_INV_DI              0x00u   /*!< Input function is not inverted */
#define IOCON_PIO_MODE_INACT          0x00u   /*!< No addition pin function */
#define IOCON_PIO_MODE_PULLUP         0x20u   /*!< Selects pull-up function */
#define IOCON_PIO_OPENDRAIN_DI        0x00u   /*!< Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD       0x00u   /*!< Standard mode, output slew rate control is enabled */
#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port 1 */
#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port 1 */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port 1 */
#define PIN05_IDX                        5u   /*!< Pin number for pin 5 in a port 0 */
#define PIN22_IDX                       22u   /*!< Pin number for pin 22 in a port 0 */
#define PIN26_IDX                       26u   /*!< Pin number for pin 26 in a port 0 */
#define PIN27_IDX                       27u   /*!< Pin number for pin 27 in a port 0 */
#define PIN29_IDX                       29u   /*!< Pin number for pin 29 in a port 0 */
#define PIN30_IDX                       30u   /*!< Pin number for pin 30 in a port 0 */
#define PIN0_IDX                         0u   /*!< Pin number for pin 00 in a port 2 */
#define PIN18_IDX                       18u   /*!< Pin number for pin 18 in a port 2 */
#define PIN19_IDX                       19u   /*!< Pin number for pin 19 in a port 2 */
#define PIN20_IDX                       20u   /*!< Pin number for pin 20 in a port 2 */
#define PIN11_IDX                       11u   /*!< Pin number for pin 11 in a port 3 */
#define PIN13_IDX                       13u   /*!< Pin number for pin 13 in a port 3 */
#define PIN21_IDX                       21u   /*!< Pin number for pin 21 in a port 3 */
#define PIN23_IDX                       23u   /*!< Pin number for pin 23 in a port 3 */
#define PIN24_IDX                       24u   /*!< Pin number for pin 24 in a port 3 */
#define PIN01_IDX                        1u   /*!< Pin number for pin 1 in a port 4 */
#define PIN02_IDX                        2u   /*!< Pin number for pin 2 in a port 4 */
#define PIN03_IDX                        3u   /*!< Pin number for pin 3 in a port 4 */
#define PIN06_IDX                        6u   /*!< Pin number for pin 6 in a port 4 */
#define PIN8_IDX                         8u   /*!< Pin number for pin 8 in a port 4 */

#define PORT0_IDX                        0u   /*!< Port index */
#define PORT1_IDX                        1u   /*!< Port index */
#define PORT2_IDX                        2u   /*!< Port index */
#define PORT3_IDX                        3u   /*!< Port index */
#define PORT4_IDX                        4u   /*!< Port index */


/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: A2, peripheral: FLEXCOMM0, signal: TXD_SCL_MISO, pin_signal: PIO0_30/FC0_TXD_SCL_MISO/CTIMER0_MAT0/SCT0_OUT9/TRACEDATA(1), mode: inactive, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
  - {pin_num: B13, peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI, pin_signal: PIO0_29/FC0_RXD_SDA_MOSI/CTIMER2_MAT3/SCT0_OUT8/TRACEDATA(2), mode: inactive, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
  - {pin_num: L14, peripheral: DMIC0, signal: 'CLK, 1', pin_signal: PIO1_2/CAN0_TD/CTIMER0_MAT3/SCT0_GPI6/PDM1_CLK/FC10_TXD_SCL_MISO/USB1_PORTPWRN, mode: pullUp,
    invert: disabled, glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
  - {pin_num: J13, peripheral: DMIC0, signal: 'DATA, 1', pin_signal: PIO1_3/CAN0_RD/SCT0_OUT4/PDM1_DATA/USB0_PORTPWRN/FC10_SCK, mode: pullUp, invert: disabled, glitch_filter: disabled,
    slew_rate: standard, open_drain: disabled}
  - {pin_num: B12, peripheral: USBFSH, signal: USB_VBUS, pin_signal: PIO0_22/FC6_TXD_SCL_MISO_WS/UTICK_CAP1/CTIMER3_CAP3/SCT0_OUT3/USB0_VBUS, mode: inactive, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 *
 *END**************************************************************************/
#define IOCON_PIO_DIGITAL_EN 0x0100u    /*!<@brief Enables digital function */
#define IOCON_PIO_FUNC0 0x00u           /*!<@brief Selects pin function 0 */
#define IOCON_PIO_FUNC1 0x01u           /*!<@brief Selects pin function 1 */
#define IOCON_PIO_FUNC2 0x02u           /*!<@brief Selects pin function 2 */
#define IOCON_PIO_FUNC3 0x03u           /*!<@brief Selects pin function 3 */
#define IOCON_PIO_I2CDRIVE_HIGH 0x0400u /*!<@brief High drive: 20 mA */
#define IOCON_PIO_I2CFILTER_DI 0x0800u  /*!<@brief I2C 50 ns glitch filter disabled */
#define IOCON_PIO_I2CSLEW_I2C 0x00u     /*!<@brief I2C mode */

#define IOCON_PIO_I2CDRIVE_LOW        0x00u   /*!< Low drive: 4 mA */
//#define IOCON_PIO_I2CFILTER_DI      0x0800u   /*!< I2C 50 ns glitch filter disabled */
#define IOCON_PIO_I2CSLEW_GPIO        0x40u   /*!< GPIO mode */

#define IOCON_PIO_INPFILT_OFF 0x0200u   /*!<@brief Input filter disabled */
#define IOCON_PIO_INV_DI 0x00u          /*!<@brief Input function is not inverted */
#define IOCON_PIO_MODE_INACT 0x00u      /*!<@brief No addition pin function */
#define IOCON_PIO_MODE_PULLUP 0x20u     /*!<@brief Selects pull-up function */
#define IOCON_PIO_OPENDRAIN_DI 0x00u    /*!<@brief Open drain is disabled */
#define IOCON_PIO_SLEW_FAST 0x0400u     /*!<@brief Fast mode, slew rate control is disabled */
#define IOCON_PIO_SLEW_STANDARD 0x00u   /*!<@brief Standard mode, output slew rate control is enabled */

void BOARD_InitPins(void) { /* Function assigned for the Core #0 (ARM Cortex-M4) */
  CLOCK_EnableClock(kCLOCK_Iocon);                           /* Enables the clock for the IOCON block. 0 = Disable; 1 = Enable.: 0x01u */

  const uint32_t port0_pin22_config = (
    IOCON_PIO_FUNC7 |                                        /* Pin is configured as USB0_VBUS */
    IOCON_PIO_MODE_INACT |                                   /* No addition pin function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN22_IDX, port0_pin22_config); /* PORT0 PIN22 (coords: B12) is configured as USB0_VBUS */
  const uint32_t port0_pin29_config = (
    IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC0_RXD_SDA_MOSI */
    IOCON_PIO_MODE_INACT |                                   /* No addition pin function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );

  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN29_IDX, port0_pin29_config); /* PORT0 PIN29 (coords: B13) is configured as FC0_RXD_SDA_MOSI */
  const uint32_t port0_pin30_config = (
    IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC0_TXD_SCL_MISO */
    IOCON_PIO_MODE_INACT |                                   /* No addition pin function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );

  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN30_IDX, port0_pin30_config); /* PORT0 PIN30 (coords: A2) is configured as FC0_TXD_SCL_MISO */
  const uint32_t port1_pin2_config = (
    IOCON_PIO_FUNC5 |                                        /* Pin is configured as PDM1_CLK */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT1_IDX, PIN2_IDX, port1_pin2_config); /* PORT1 PIN2 (coords: L14) is configured as PDM1_CLK */
  const uint32_t port1_pin3_config = (
    IOCON_PIO_FUNC5 |                                        /* Pin is configured as PDM1_DATA */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT1_IDX, PIN3_IDX, port1_pin3_config); /* PORT1 PIN3 (coords: J13) is configured as PDM1_DATA */


 const uint32_t port2_pin18_config = (
	IOCON_PIO_FUNC3 |                                        /* Pin is configured as FC7_SCK */
	IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
	IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	IOCON_PIO_SLEW_FAST |                                    /* Fast mode, slew rate control is disabled */
	IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
 /* PORT2 PIN18 (coords: N10) is configured as FC7_SCK */
 IOCON_PinMuxSet(IOCON, PORT2_IDX, PIN18_IDX, port2_pin18_config);

 const uint32_t port2_pin19_config = (
	IOCON_PIO_FUNC3 |                                       /* Pin is configured as FC7_RXD_SDA_MOSI_DATA */
	IOCON_PIO_MODE_PULLUP |                                 /* Selects pull-up function */
	IOCON_PIO_INV_DI  |                                     /* Input function is not inverted */
	IOCON_PIO_DIGITAL_EN |                                  /* Enables digital function */
	IOCON_PIO_INPFILT_OFF |                                 /* Input filter disabled */
	IOCON_PIO_SLEW_FAST |                                   /* Fast mode, slew rate control is disabled */
	IOCON_PIO_OPENDRAIN_DI                                  /* Open drain is disabled */
 );
 /* PORT2 PIN19 (coords: P12) is configured as FC7_RXD_SDA_MOSI_DATA */
 IOCON_PinMuxSet(IOCON, PORT2_IDX, PIN19_IDX, port2_pin19_config);

 const uint32_t port2_pin20_config = (
	IOCON_PIO_FUNC3 |                                       /* Pin is configured as FC7_TXD_SCL_MISO_WS */
	IOCON_PIO_MODE_PULLUP |                                 /* Selects pull-up function */
	IOCON_PIO_INV_DI |                                      /* Input function is not inverted */
	IOCON_PIO_DIGITAL_EN |                                  /* Enables digital function */
	IOCON_PIO_INPFILT_OFF |                                 /* Input filter disabled */
	IOCON_PIO_SLEW_FAST |                                   /* Fast mode, slew rate control is disabled */
	IOCON_PIO_OPENDRAIN_DI                                  /* Open drain is disabled */
 );
 /* PORT2 PIN20 (coords: P13) is configured as FC7_TXD_SCL_MISO_WS */
 IOCON_PinMuxSet(IOCON, PORT2_IDX, PIN20_IDX, port2_pin20_config);

 const uint32_t port3_pin11_config = (
	IOCON_PIO_FUNC1 |                                       /* Pin is configured as MCLK */
	IOCON_PIO_MODE_INACT |                                  /* No addition pin function */
	IOCON_PIO_INV_DI |                                      /* Input function is not inverted */
	IOCON_PIO_DIGITAL_EN |                                  /* Enables digital function */
	IOCON_PIO_INPFILT_OFF |                                 /* Input filter disabled */
	IOCON_PIO_SLEW_STANDARD |                               /* Standard mode, output slew rate control is enabled */
	IOCON_PIO_OPENDRAIN_DI                                  /* Open drain is disabled */
 );
 /* PORT3 PIN11 (coords: B2) is configured as MCLK */
 IOCON_PinMuxSet(IOCON, PORT3_IDX, PIN11_IDX, port3_pin11_config);

#if 0
/* const uint32_t port3_pin23_config = (
	IOCON_PIO_FUNC1 |                                       /* Pin is configured as FC2_CTS_SDA_SSEL0 */
	IOCON_PIO_I2CSLEW_I2C |                                 /* I2C mode */
	IOCON_PIO_INV_DI |                                      /* Input function is not inverted */
	IOCON_PIO_DIGITAL_EN |                                  /* Enables digital function */
	IOCON_PIO_INPFILT_OFF |                                 /* Input filter disabled */
	IOCON_PIO_I2CDRIVE_HIGH |                               /* High drive: 20 mA */
	IOCON_PIO_I2CFILTER_DI                                  /* I2C 50 ns glitch filter disabled */
 );
 /* PORT3 PIN23 (coords: C2) is configured as FC2_CTS_SDA_SSEL0 */
 IOCON_PinMuxSet(IOCON, PORT3_IDX, PIN23_IDX, port3_pin23_config);

 const uint32_t port3_pin24_config = (
	IOCON_PIO_FUNC1 |                                       /* Pin is configured as FC2_RTS_SCL_SSEL1 */
	IOCON_PIO_I2CSLEW_I2C |                                 /* I2C mode */
	IOCON_PIO_INV_DI |                                      /* Input function is not inverted */
	IOCON_PIO_DIGITAL_EN |                                  /* Enables digital function */
	IOCON_PIO_INPFILT_OFF |                                 /* Input filter disabled */
	IOCON_PIO_I2CDRIVE_HIGH |                               /* High drive: 20 mA */
	IOCON_PIO_I2CFILTER_DI                                  /* I2C 50 ns glitch filter disabled */
 );

 /* PORT3 PIN24 (coords: E2) is configured as FC2_RTS_SCL_SSEL1 */
 IOCON_PinMuxSet(IOCON, PORT3_IDX, PIN24_IDX, port3_pin24_config);

#endif

 const uint32_t port4_pin1_config = (
	IOCON_PIO_FUNC2 |                                        /* Pin is configured as FC6_SCK */
	IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
	IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	IOCON_PIO_SLEW_FAST |                                    /* Fast mode, slew rate control is disabled */
	IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
 );
 /* PORT4 PIN1 (coords: G14) is configured as FC6_SCK */
 IOCON_PinMuxSet(IOCON, PORT4_IDX, PIN01_IDX, port4_pin1_config);

 const uint32_t port4_pin2_config = (
	 IOCON_PIO_FUNC2 |                                       /* Pin is configured as FC6_RXD_SDA_MOSI_DATA */
	 IOCON_PIO_MODE_PULLUP |                                 /* Selects pull-up function */
	 IOCON_PIO_INV_DI |                                      /* Input function is not inverted */
	 IOCON_PIO_DIGITAL_EN |                                  /* Enables digital function */
	 IOCON_PIO_INPFILT_OFF |                                 /* Input filter disabled */
	 IOCON_PIO_SLEW_FAST |                                   /* Fast mode, slew rate control is disabled */
	 IOCON_PIO_OPENDRAIN_DI                                  /* Open drain is disabled */
 );
 /* PORT4 PIN2 (coords: F14) is configured as FC6_RXD_SDA_MOSI_DATA */
 IOCON_PinMuxSet(IOCON, PORT4_IDX, PIN02_IDX, port4_pin2_config);

 const uint32_t port4_pin3_config = (
	IOCON_PIO_FUNC2 |                                        /* Pin is configured as FC6_TXD_SCL_MISO_WS */
	IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
	IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	IOCON_PIO_SLEW_FAST |                                    /* Fast mode, slew rate control is disabled */
	IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
 );
 /* PORT4 PIN3 (coords: F13) is configured as FC6_TXD_SCL_MISO_WS */
 IOCON_PinMuxSet(IOCON, PORT4_IDX, PIN03_IDX, port4_pin3_config);

    const uint32_t port1_pin17_config = (
		IOCON_PIO_FUNC0 |                                        /* Pin is configured as SX1278 RESET */
		IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
		IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
		IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
		IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
		IOCON_PIO_SLEW_STANDARD |                                    /* Fast mode, slew rate control is disabled */
		IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
		);
		IOCON_PinMuxSet(IOCON, PORT1_IDX, PIN17_IDX, port1_pin17_config);

	const uint32_t port0_pin05_config = (
		IOCON_PIO_FUNC0 |                                        /* Pin is configured as SX1278 TXEN */
		IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
		IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
		IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
		IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
		IOCON_PIO_SLEW_STANDARD |                                    /* Fast mode, slew rate control is disabled */
		IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
		);
		IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN05_IDX, port0_pin05_config);

	const uint32_t port3_pin13_config = (
		IOCON_PIO_FUNC0 |                                        /* Pin is configured as SX1278 RXEN */
		IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
		IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
		IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
		IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
		IOCON_PIO_SLEW_STANDARD |                                    /* Fast mode, slew rate control is disabled */
		IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
	     );
	IOCON_PinMuxSet(IOCON, PORT3_IDX, PIN13_IDX, port3_pin13_config);

	const uint32_t port1_pin18_config = (
		IOCON_PIO_FUNC0 |                                        /* Pin is configured as SX1278 DIO0 */
		IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
		IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
		IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
		IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
		IOCON_PIO_SLEW_STANDARD |                                    /* Fast mode, slew rate control is disabled */
		IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
	     );
	IOCON_PinMuxSet(IOCON, PORT1_IDX, PIN18_IDX, port1_pin18_config);

	const uint32_t port2_pin0_config = (
		IOCON_PIO_FUNC0 |                                        /* Pin is configured as SX1278 DIO2 */
		IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
		IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
		IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
		IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
		IOCON_PIO_SLEW_STANDARD |                                    /* Fast mode, slew rate control is disabled */
		IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
		 );
    IOCON_PinMuxSet(IOCON, PORT2_IDX, PIN0_IDX, port2_pin0_config);

	const uint32_t port4_pin06_config = (
		IOCON_PIO_FUNC0 |                                        /* Pin is configured as FC9_CTS_SDA_SSEL0 */
		IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
		IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
		IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
		IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
		IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
		IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
		);
	IOCON_PinMuxSet(IOCON, PORT4_IDX, PIN06_IDX, port4_pin06_config); /* PORT3 PIN30 (coords: K13) is configured as FC9_CTS_SDA_SSEL0 */

#if 0
	const uint32_t port0_pin02_config = (
	    IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC3_MISO*/
	    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
	    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
	    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
	  );
	  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN02_IDX, port0_pin02_config); /* PORT0 PIN02 (coords: N2) is configured as FC3_MISO */
	  const uint32_t port0_pin03_config = (
	    IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC3_MOSI */
	    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
	    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
	  );
	  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN03_IDX, port0_pin03_config); /* PORT0 PIN03 (coords: P5) is configured as FC3_MOSI */
	  const uint32_t port0_pin0_config = (
	    IOCON_PIO_FUNC2 |                                        /* Pin is configured as FC3_CLK */
	    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
	    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
	  );
	  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN0_IDX, port0_pin0_config); /* PORT0 PIN0 (coords: N5) is configured as FC3_CLK */

	  const uint32_t port0_pin01_config = (
	  		IOCON_PIO_FUNC0 |                                        /* Pin is configured as FC9_CTS_SDA_SSEL0 */
	  		IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
	  		IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	  		IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	  		IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	  		IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
	  		IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
	  		);
	  	IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN01_IDX, port0_pin01_config); /* PORT0 PIN01 (coords: K13) is configured as FC9_CTS_SDA_SSEL0 */
#endif

	    const uint32_t port0_pin26_config = (
	      IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC2_RXD_SDA_MOSI */
	      IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
	      IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	      IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	      IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	      IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
	      IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
	    );
	    IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN26_IDX, port0_pin26_config); /* PORT0 PIN26 (coords: M13) is configured as FC2_RXD_SDA_MOSI */
	    const uint32_t port0_pin27_config = (
	      IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC2_TXD_SCL_MISO */
	      IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
	      IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	      IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	      IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	      IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
	      IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
	    );
	    IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN27_IDX, port0_pin27_config); /* PORT0 PIN27 (coords: L9) is configured as FC2_TXD_SCL_MISO */
	    const uint32_t port3_pin23_config = (
	      IOCON_PIO_FUNC0 |                                        /* Pin is configured as FC2_CTS_SDA_SSEL0 */
	      IOCON_PIO_I2CSLEW_GPIO |                                 /* GPIO mode */
	      IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	      IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	      IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	      IOCON_PIO_I2CDRIVE_LOW |                                 /* Low drive: 4 mA */
	      IOCON_PIO_I2CFILTER_DI                                   /* I2C 50 ns glitch filter disabled */
	    );
	    IOCON_PinMuxSet(IOCON, PORT3_IDX, PIN23_IDX, port3_pin23_config); /* PORT3 PIN23 (coords: C2) is configured as FC2_CTS_SDA_SSEL0 */
	    const uint32_t port4_pin8_config = (
	        IOCON_PIO_FUNC2 |                                        /* Pin is configured as FC2_SCK */
	        IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
	        IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
	        IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
	        IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
	        IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
	        IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
	      );
	      IOCON_PinMuxSet(IOCON, PORT4_IDX, PIN8_IDX, port4_pin8_config); /* PORT4 PIN8 (coords: B14) is configured as FC2_SCK */
}
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
SPI9_InitPins:
- options: {callFromInitBoot: 'false', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: N2, peripheral: FLEXCOMM9, signal: SCK, pin_signal: PIO3_20/FC9_SCK/SD_CARD_INT_N/CLKOUT/SCT0_OUT7}
  - {pin_num: P5, peripheral: FLEXCOMM9, signal: RXD_SDA_MOSI, pin_signal: PIO3_21/FC9_RXD_SDA_MOSI/SD_BACKEND_PWR/CTIMER4_MAT3/UTICK_CAP2/ADC0_9}
  - {pin_num: N5, peripheral: FLEXCOMM9, signal: TXD_SCL_MISO, pin_signal: PIO3_22/FC9_TXD_SCL_MISO/ADC0_10}
  - {pin_num: K13, peripheral: FLEXCOMM9, signal: CTS_SDA_SSEL0, pin_signal: PIO3_30/FC9_CTS_SDA_SSEL0/SCT0_OUT4/FC4_SSEL2/EMC_A(19)}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : SPI9_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M4F */
void SPI9_InitPins(void)
{
    /* Enables the clock for the IOCON block. 0 = Disable; 1 = Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Iocon);

    IOCON->PIO[3][20] = ((IOCON->PIO[3][20] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT320 (pin N2) is configured as FC9_SCK. */
                         | IOCON_PIO_FUNC(PIO320_FUNC_ALT1)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO320_DIGIMODE_DIGITAL));

    IOCON->PIO[3][21] = ((IOCON->PIO[3][21] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT321 (pin P5) is configured as FC9_RXD_SDA_MOSI. */
                         | IOCON_PIO_FUNC(PIO321_FUNC_ALT1)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO321_DIGIMODE_DIGITAL));

    IOCON->PIO[3][22] = ((IOCON->PIO[3][22] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT322 (pin N5) is configured as FC9_TXD_SCL_MISO. */
                         | IOCON_PIO_FUNC(PIO322_FUNC_ALT1)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO322_DIGIMODE_DIGITAL));

#if 0
    IOCON->PIO[3][30] = ((IOCON->PIO[3][30] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT330 (pin K13) is configured as FC9_CTS_SDA_SSEL0. */
                         | IOCON_PIO_FUNC(PIO330_FUNC_ALT1)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO330_DIGIMODE_DIGITAL));
#endif
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
SPI9_DeinitPins:
- options: {callFromInitBoot: 'false', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: N2, peripheral: GPIO, signal: 'PIO3, 20', pin_signal: PIO3_20/FC9_SCK/SD_CARD_INT_N/CLKOUT/SCT0_OUT7}
  - {pin_num: N5, peripheral: GPIO, signal: 'PIO3, 22', pin_signal: PIO3_22/FC9_TXD_SCL_MISO/ADC0_10}
  - {pin_num: P5, peripheral: GPIO, signal: 'PIO3, 21', pin_signal: PIO3_21/FC9_RXD_SDA_MOSI/SD_BACKEND_PWR/CTIMER4_MAT3/UTICK_CAP2/ADC0_9}
  - {pin_num: K13, peripheral: GPIO, signal: 'PIO3, 30', pin_signal: PIO3_30/FC9_CTS_SDA_SSEL0/SCT0_OUT4/FC4_SSEL2/EMC_A(19)}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : SPI9_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M4F */
void SPI9_DeinitPins(void)
{
    /* Enables the clock for the IOCON block. 0 = Disable; 1 = Enable.: 0x01u */
    CLOCK_EnableClock(kCLOCK_Iocon);

    IOCON->PIO[3][20] = ((IOCON->PIO[3][20] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT320 (pin N2) is configured as PIO3_20. */
                         | IOCON_PIO_FUNC(PIO320_FUNC_ALT0)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO320_DIGIMODE_DIGITAL));

    IOCON->PIO[3][21] = ((IOCON->PIO[3][21] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT321 (pin P5) is configured as PIO3_21. */
                         | IOCON_PIO_FUNC(PIO321_FUNC_ALT0)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO321_DIGIMODE_DIGITAL));

    IOCON->PIO[3][22] = ((IOCON->PIO[3][22] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT322 (pin N5) is configured as PIO3_22. */
                         | IOCON_PIO_FUNC(PIO322_FUNC_ALT0)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO322_DIGIMODE_DIGITAL));

    IOCON->PIO[3][30] = ((IOCON->PIO[3][30] &
                          /* Mask bits to zero which are setting */
                          (~(IOCON_PIO_FUNC_MASK | IOCON_PIO_DIGIMODE_MASK)))

                         /* Selects pin function.
                          * : PORT330 (pin K13) is configured as PIO3_30. */
                         | IOCON_PIO_FUNC(PIO330_FUNC_ALT0)

                         /* Select Analog/Digital mode.
                          * : Digital mode. */
                         | IOCON_PIO_DIGIMODE(PIO330_DIGIMODE_DIGITAL));
}
/*******************************************************************************
 * EOF
 ******************************************************************************/
