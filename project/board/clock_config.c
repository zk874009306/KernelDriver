/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * How to set up clock using clock driver functions:
 *
 * 1. Setup clock sources.
 *
 * 2. Setup voltage for the fastest of the clock outputs
 *
 * 3. Set up wait states of the flash.
 *
 * 4. Set up all dividers.
 *
 * 5. Set up all selectors to provide selected clocks.
 */

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v5.0
processor: LPC54018
package_id: LPC54018JET180
mcu_data: ksdk2_0
processor_version: 0.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

#include "fsl_power.h"
#include "fsl_clock.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_BootClockPLL180M();
}

/*******************************************************************************
 ******************** Configuration BOARD_BootClockFRO12M **********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockFRO12M
outputs:
- {id: FRO12M_clock.outFreq, value: 12 MHz}
- {id: FROHF_clock.outFreq, value: 48 MHz}
- {id: MAIN_clock.outFreq, value: 12 MHz}
- {id: System_clock.outFreq, value: 12 MHz}
settings:
- {id: SYSCON.EMCCLKDIV.scale, value: '1', locked: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockFRO12M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockFRO12M configuration
 ******************************************************************************/
void BOARD_BootClockFRO12M(void)
{
    /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_EN);                   /*!< Ensure FRO is on  */
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                  /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                                                being below the voltage for current speed */
    /*!< Need to make sure ROM and OTP has power(PDRUNCFG0[17,29]= 0U) 
         before calling this API since this API is implemented in ROM code */
    CLOCK_SetupFROClocking(12000000U);                    /*!< Set up FRO to the 12 MHz, just for sure */
    POWER_SetVoltageForFreq(12000000U);             /*!< Set voltage for the one of the fastest clock outputs: System clock output */

    /*!< Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);                  /*!< Reset divider counter and set divider to value 1 */

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                     /*!< Switch MAIN_CLK to FRO12M */
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKFRO12M_CORE_CLOCK;
}

/*******************************************************************************
 ******************* Configuration BOARD_BootClockFROHF48M *********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockFROHF48M
outputs:
- {id: FRO12M_clock.outFreq, value: 12 MHz}
- {id: FROHF_clock.outFreq, value: 48 MHz}
- {id: MAIN_clock.outFreq, value: 48 MHz}
- {id: System_clock.outFreq, value: 48 MHz}
settings:
- {id: SYSCON.MAINCLKSELA.sel, value: SYSCON.fro_hf}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockFROHF48M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockFROHF48M configuration
 ******************************************************************************/
void BOARD_BootClockFROHF48M(void)
{
    /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_EN);                   /*!< Ensure FRO is on  */
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                  /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                                                being below the voltage for current speed */
    POWER_SetVoltageForFreq(48000000U);             /*!< Set voltage for the one of the fastest clock outputs: System clock output */
    /*!< Need to make sure ROM and OTP has power(PDRUNCFG0[17,29]= 0U) 
         before calling this API since this API is implemented in ROM code */
    CLOCK_SetupFROClocking(48000000U);              /*!< Set up high frequency FRO output to selected frequency */

    /*!< Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);                  /*!< Reset divider counter and set divider to value 1 */

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kFRO_HF_to_MAIN_CLK);                     /*!< Switch MAIN_CLK to FRO_HF */
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKFROHF48M_CORE_CLOCK;
}

/*******************************************************************************
 ******************* Configuration BOARD_BootClockFROHF96M *********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockFROHF96M
outputs:
- {id: FRO12M_clock.outFreq, value: 12 MHz}
- {id: FROHF_clock.outFreq, value: 96 MHz}
- {id: MAIN_clock.outFreq, value: 96 MHz}
- {id: System_clock.outFreq, value: 96 MHz}
settings:
- {id: SYSCON.MAINCLKSELA.sel, value: SYSCON.fro_hf}
sources:
- {id: SYSCON.fro_hf.outFreq, value: 96 MHz}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockFROHF96M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockFROHF96M configuration
 ******************************************************************************/
void BOARD_BootClockFROHF96M(void)
{
    /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_EN);                   /*!< Ensure FRO is on  */
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                  /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                                                being below the voltage for current speed */
    POWER_SetVoltageForFreq(96000000U);             /*!< Set voltage for the one of the fastest clock outputs: System clock output */
    /*!< Need to make sure ROM and OTP has power(PDRUNCFG0[17,29]= 0U) 
         before calling this API since this API is implemented in ROM code */
    CLOCK_SetupFROClocking(96000000U);              /*!< Set up high frequency FRO output to selected frequency */

    /*!< Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);                  /*!< Reset divider counter and set divider to value 1 */

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kFRO_HF_to_MAIN_CLK);                     /*!< Switch MAIN_CLK to FRO_HF */
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKFROHF96M_CORE_CLOCK;
}

/*******************************************************************************
 ******************** Configuration BOARD_BootClockPLL180M *********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockPLL180M
called_from_default_init: true
outputs:
- {id: FRO12M_clock.outFreq, value: 12 MHz}
- {id: FROHF_clock.outFreq, value: 96 MHz}
- {id: MAIN_clock.outFreq, value: 180 MHz}
- {id: SYSPLL_clock.outFreq, value: 180 MHz}
- {id: System_clock.outFreq, value: 180 MHz}
- {id: USB0_clock.outFreq, value: 96 MHz}
settings:
- {id: SYSCON.MAINCLKSELB.sel, value: SYSCON.PLL_BYPASS}
- {id: SYSCON.M_MULT.scale, value: '30', locked: true}
- {id: SYSCON.N_DIV.scale, value: '1', locked: true}
- {id: SYSCON.PDEC.scale, value: '2', locked: true}
- {id: SYSCON.USB0CLKSEL.sel, value: SYSCON.fro_hf}
- {id: SYSCON_PDRUNCFG0_PDEN_SYS_PLL_CFG, value: Power_up}
sources:
- {id: SYSCON._clk_in.outFreq, value: 12 MHz, enabled: true}
- {id: SYSCON.fro_hf.outFreq, value: 96 MHz}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockPLL180M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockPLL180M configuration
 ******************************************************************************/
void BOARD_BootClockPLL180M(void)
{
    /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_EN);                   /*!< Ensure FRO is on  */
    CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                  /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                                                being below the voltage for current speed */
    POWER_DisablePD(kPDRUNCFG_PD_SYS_OSC);          /*!< Enable System Oscillator Power */
    SYSCON->SYSOSCCTRL = ((SYSCON->SYSOSCCTRL & ~SYSCON_SYSOSCCTRL_FREQRANGE_MASK) | SYSCON_SYSOSCCTRL_FREQRANGE(0U)); /*!< Set system oscillator range */
    POWER_SetVoltageForFreq(180000000U);             /*!< Set voltage for the one of the fastest clock outputs: System clock output */
    /*!< Set up SYS PLL */
    const pll_setup_t pllSetup = {
        .pllctrl =  SYSCON_SYSPLLCTRL_SELI(32U) | SYSCON_SYSPLLCTRL_SELP(16U) | SYSCON_SYSPLLCTRL_SELR(0U),
        .pllmdec = (SYSCON_SYSPLLMDEC_MDEC(8191U)),
        .pllndec = (SYSCON_SYSPLLNDEC_NDEC(770U)),
        .pllpdec = (SYSCON_SYSPLLPDEC_PDEC(98U)),
        .pllRate = 180000000U,
        .flags =  PLL_SETUPFLAG_WAITLOCK | PLL_SETUPFLAG_POWERUP
    };
    CLOCK_AttachClk(kFRO12M_to_SYS_PLL);        /*!< Set sys pll clock source*/
    CLOCK_SetPLLFreq(&pllSetup);                 /*!< Configure PLL to the desired value */
    /*!< Need to make sure ROM and OTP has power(PDRUNCFG0[17,29]= 0U) 
         before calling this API since this API is implemented in ROM code */
    CLOCK_SetupFROClocking(96000000U);              /*!< Set up high frequency FRO output to selected frequency */

    /*!< Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);                  /*!< Reset divider counter and set divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivUsb0Clk, 0U, true);                  /*!< Reset USB0CLKDIV divider counter and halt it */
    CLOCK_SetClkDiv(kCLOCK_DivUsb0Clk, 1U, false);                  /*!< Set USB0CLKDIV divider to value 1 */

    /*!< Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kSYS_PLL_to_MAIN_CLK);                  /*!< Switch MAIN_CLK to SYS_PLL */
    CLOCK_AttachClk(kFRO_HF_to_USB0_CLK);                  /*!< Switch USB0_CLK to FRO_HF */
    SYSCON->MAINCLKSELA = ((SYSCON->MAINCLKSELA & ~SYSCON_MAINCLKSELA_SEL_MASK) | SYSCON_MAINCLKSELA_SEL(0U)); /*!< Switch MAINCLKSELA to FRO12M even it is not used for MAINCLKSELB */
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKPLL180M_CORE_CLOCK;
}

