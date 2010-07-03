/* arch/arm/mach-msm/proc_comm.h
 *
 * Copyright (c) 2007 QUALCOMM Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ARCH_ARM_MACH_MSM_MSM_PROC_COMM_H_
#define _ARCH_ARM_MACH_MSM_MSM_PROC_COMM_H_

enum {
	PCOM_CMD_IDLE = 0x0,
	PCOM_CMD_DONE,
	PCOM_RESET_APPS,
	PCOM_RESET_CHIP,
	PCOM_CONFIG_NAND_MPU,
	PCOM_CONFIG_USB_CLKS,
	PCOM_GET_POWER_ON_STATUS,
	PCOM_GET_WAKE_UP_STATUS,
	PCOM_GET_BATT_LEVEL,
	PCOM_CHG_IS_CHARGING,
	PCOM_POWER_DOWN,
	PCOM_USB_PIN_CONFIG,
	PCOM_USB_PIN_SEL,
	PCOM_SET_RTC_ALARM,
	PCOM_NV_READ,
	PCOM_NV_WRITE,
#ifdef CONFIG_MACH_CHUCK
	PCOM_GET_PALM_HW_CFG,
	PCOM_ASSIGN_RDM_SRVC,
#else
	PCOM_GET_UUID_HIGH,
	PCOM_GET_UUID_LOW,
#endif
	PCOM_GET_HW_ENTROPY,
	PCOM_RPC_GPIO_TLMM_CONFIG_REMOTE,
	PCOM_CLKCTL_RPC_ENABLE,
	PCOM_CLKCTL_RPC_DISABLE,
	PCOM_CLKCTL_RPC_RESET,
	PCOM_CLKCTL_RPC_SET_FLAGS,
	PCOM_CLKCTL_RPC_SET_RATE,
	PCOM_CLKCTL_RPC_MIN_RATE,
	PCOM_CLKCTL_RPC_MAX_RATE,
	PCOM_CLKCTL_RPC_RATE,
	PCOM_CLKCTL_RPC_PLL_REQUEST,
	PCOM_CLKCTL_RPC_ENABLED,
	PCOM_VREG_SWITCH,
	PCOM_VREG_SET_LEVEL,
	PCOM_GPIO_TLMM_CONFIG_GROUP,
	PCOM_GPIO_TLMM_UNCONFIG_GROUP,
	PCOM_NV_WRITE_BYTES_4_7,
	PCOM_CONFIG_DISP,
	PCOM_GET_FTM_BOOT_COUNT,
	PCOM_RPC_GPIO_TLMM_CONFIG_EX,
	PCOM_PM_MPP_CONFIG,
	PCOM_GPIO_IN,
	PCOM_GPIO_OUT,
	PCOM_RESET_MODEM,
	PCOM_RESET_CHIP_IMM,
	PROC_COMM_RESET_CRCI_DBG,
	PCOM_VREG_PULLDOWN,
	PCOM_GET_MODEM_VERSION,
	PCOM_CLK_REGIME_SEC_RESET,
	PCOM_CLK_REGIME_SEC_RESET_ASSERT,
	PCOM_CLK_REGIME_SEC_RESET_DEASSERT,
	PCOM_CLK_REGIME_SEC_PLL_REQUEST_WRP,
	PCOM_CLK_REGIME_SEC_ENABLE,
	PCOM_CLK_REGIME_SEC_DISABLE,
	PCOM_CLK_REGIME_SEC_IS_ON,
	PCOM_CLK_REGIME_SEC_SEL_CLK_INV,
	PCOM_CLK_REGIME_SEC_SEL_CLK_SRC,
	PCOM_CLK_REGIME_SEC_SEL_CLK_DIV,
	PCOM_CLK_REGIME_SEC_ICODEC_CLK_ENABLE,
	PCOM_CLK_REGIME_SEC_ICODEC_CLK_DISABLE,
	PCOM_CLK_REGIME_SEC_SEL_SPEED,
	PCOM_CLK_REGIME_SEC_CONFIG_GP_CLK_WRP,
	PCOM_CLK_REGIME_SEC_CONFIG_MDH_CLK_WRP,
	PCOM_CLK_REGIME_SEC_USB_XTAL_ON,
	PCOM_CLK_REGIME_SEC_USB_XTAL_OFF,
	PCOM_CLK_REGIME_SEC_SET_QDSP_DME_MODE,
	PCOM_CLK_REGIME_SEC_SWITCH_ADSP_CLK,
	PCOM_CLK_REGIME_SEC_GET_MAX_ADSP_CLK_KHZ,
	PCOM_CLK_REGIME_SEC_GET_I2C_CLK_KHZ,
	PCOM_CLK_REGIME_SEC_MSM_GET_CLK_FREQ_KHZ,
	PCOM_CLK_REGIME_SEC_SEL_VFE_SRC,
	PCOM_CLK_REGIME_SEC_MSM_SEL_CAMCLK,
	PCOM_CLK_REGIME_SEC_MSM_SEL_LCDCLK,
	PCOM_CLK_REGIME_SEC_VFE_RAIL_OFF,
	PCOM_CLK_REGIME_SEC_VFE_RAIL_ON,
	PCOM_CLK_REGIME_SEC_GRP_RAIL_OFF,
	PCOM_CLK_REGIME_SEC_GRP_RAIL_ON,
	PCOM_CLK_REGIME_SEC_VDC_RAIL_OFF,
	PCOM_CLK_REGIME_SEC_VDC_RAIL_ON,
	PCOM_CLK_REGIME_SEC_LCD_CTRL,
	PCOM_CLK_REGIME_SEC_REGISTER_FOR_CPU_RESOURCE,
	PCOM_CLK_REGIME_SEC_DEREGISTER_FOR_CPU_RESOURCE,
	PCOM_CLK_REGIME_SEC_RESOURCE_REQUEST_WRP,
	PCOM_CLK_REGIME_MSM_SEC_SEL_CLK_OWNER,
	PCOM_CLK_REGIME_SEC_DEVMAN_REQUEST_WRP,
	PCOM_GPIO_CONFIG,
	PCOM_GPIO_CONFIGURE_GROUP,
	PCOM_GPIO_TLMM_SET_PORT, 
	PCOM_GPIO_TLMM_CONFIG_EX,
	PCOM_SET_FTM_BOOT_COUNT,
	PCOM_RESERVED0,      /* smem_pcmod.c: used to test proc comm */
	PCOM_RESERVED1,      /* smem_pcmod.c: used to test proc comm */
	PCOM_PALM_DIAG_CMDS,
	PCOM_CUST02,
	PCOM_CUST03,
	
	PCOM_CLK_REGIME_ENTER_APPSBL_CHG_MODE,
	PCOM_CLK_REGIME_EXIT_APPSBL_CHG_MODE,
	PCOM_CUST04,
	PCOM_CUST05,
 
	PCOM_NUM_CMDS,
};

enum {
	 PCOM_INVALID_STATUS = 0x0,
	 PCOM_READY,
	 PCOM_CMD_RUNNING,
	 PCOM_CMD_SUCCESS,
	 PCOM_CMD_FAIL,
};

/*
 * This defines the desired MPPs setting
 */
typedef enum
{
	PCOMM_PM_MPP_CFG_OUT = 0x0,
	PCOMM_PM_MPP_CFG_IN,
	PCOMM_PM_MPP_CFG_SINK,
 	PCOMM_PM_MPP_CFG_FLASH_SINK,
	PCOMM_PM_MPP_CFG_AOUT,
	PCOMM_PM_MPP_CFG_INVALID,
} pcomm_pm_mpp_config_t;

/* 
 * Which MPP needs to be configured 
 */
typedef enum
{
	PCOMM_PM_MPP_1,
	PCOMM_PM_MPP_2,
	PCOMM_PM_MPP_3,
	PCOMM_PM_MPP_4,
	PCOMM_PM_MPP_5,
	PCOMM_PM_MPP_6,
	PCOMM_PM_MPP_7,
	PCOMM_PM_MPP_8,
	PCOMM_PM_MPP_9,
	PCOMM_PM_MPP_10,
	PCOMM_PM_MPP_11,
	PCOMM_PM_MPP_12,
	PCOMM_PM_MPP_13,
	PCOMM_PM_MPP_14,
	PCOMM_PM_MPP_15,
	PCOMM_PM_MPP_16,
	PCOMM_PM_MPP_17,
	PCOMM_PM_MPP_18,
	PCOMM_PM_MPP_19,
	PCOMM_PM_MPP_20,
	PCOMM_PM_MPP_21,
	PCOMM_PM_MPP_22,
	PCOMM_PM_MPP_INVALID,
	PCOMM_PM_NUM_MPP_HAN = PCOMM_PM_MPP_4 + 1,
	PCOMM_PM_NUM_MPP_PM7500 = PCOMM_PM_MPP_22 + 1,  /* Max number of MPP's for PM7500 */
	PCOMM_PM_NUM_MPP_PM6650 = PCOMM_PM_MPP_12 + 1,  /* Max number of MPP's for PM6650 */
	PCOMM_PM_NUM_MPP_PANORAMIX = PCOMM_PM_MPP_2 + 1,/* Max number of MPP's for PANORAMIX and PM6640 */
	PCOMM_PM_NUM_MPP_PM6640 = PCOMM_PM_NUM_MPP_PANORAMIX,
	PCOMM_PM_NUM_MPP_PM6620 = PCOMM_PM_NUM_MPP_PANORAMIX
}pcomm_pm_mpp_which_t;
 
/*
 * Analog output level of MPP.
 */
typedef enum
{     
	PCOMM_PM_MPP_AOUT_LEVEL_VREF_1p25_Volts,    
	PCOMM_PM_MPP_AOUT_LEVEL_VREF_0p625_Volts,
	PCOMM_PM_MPP_AOUT_LEVEL_VREF_2p50_Volts,
	PCOMM_PM_MPP_AOUT_LEVEL_INVALID
} pcomm_pm_mpp_aout_level_type;

/*
 * MPP aout switch type
 */
typedef enum
{
	PCOMM_PM_MPP_AOUT_SWITCH_OFF,
	PCOMM_PM_MPP_AOUT_SWITCH_ON
} pcomm_pm_mpp_aout_switch_type;

#ifdef CONFIG_MACH_CHUCK
/*
 * Pcomm for palm Diag commands
 */
typedef enum
{
	PCOMM_PALM_DIAG_CMD_DLOAD = 0,
	PCOMM_PALM_DIAG_CMD_PDOWN,
	PCOMM_PALM_DIAG_CONTROL_CHARGER,
	PCOMM_PALM_DIAG_NV_SET_LEN,
	PCOMM_PALM_DIAG_NV_SET_DATA,
	PCOMM_PALM_DIAG_NV_GET_LEN,
	PCOMM_PALM_DIAG_NV_GET_DATA,
	PCOMM_PALM_DIAG_INDUCTIVE_CHARGER_STATUS,
	PCOMM_PALM_DIAG_VREG_GET_STATUS,
	PCOMM_PALM_DIAG_VREG_GET_VOLTAGE_LVL,
	PCOMM_PALM_DIAG_GET_BANDS,
	PCOMM_PALM_DIAG_MODEM_START_ADDR,
	PCOMM_PALM_DIAG_HEADSET_STATUS,
	PCOMM_PALM_DIAG_MAX = 0xFF
} pcomm_palm_diag_cmds;


/*
 * Enum for the external entities to use for ADC channels 
 */
typedef enum
{
   PCOMM_PALM_ADC_BATT_ID,         // RAW reading: Battery ID
   PCOMM_PALM_ADC_VBATT,           // RAW reading: Main battery voltage at jack
   PCOMM_PALM_ADC_VCHG,            // RAW reading: Wall charger voltage at jack
   PCOMM_PALM_ADC_ICHG,            // RAW reading: Charger current going into the battery
   PCOMM_PALM_ADC_ICHG_OUT,        // RAW reading: Current leaving the battery
   PCOMM_PALM_ADC_HDET_PCS,        // RAW reading: HDET for PCS mode
   PCOMM_PALM_ADC_THERM,           // RAW reading: PA thermistor
   PCOMM_PALM_ADC_RES_KYPD,        //    REMOVE: Resistive keypad; not used
   PCOMM_PALM_ADC_PA_POWER_DETECT, // RAW reading: HDET0: RF power detect
   PCOMM_PALM_ADC_HDET_CELL,       // RAW reading: HDET for Cell mode
   PCOMM_PALM_ADC_HEADSET_SWITCH,  //    REMOVE: Headset switch detection; not used
   PCOMM_PALM_ADC_MSM_THERM,       // Unconnected: MSM thermistor
   // Converted values
   PCOMM_PALM_ADC_VCOIN_MV,        // Coin cell battery/capacitor voltage
   PCOMM_PALM_ADC_VBATT_MV,        // Main battery Voltage
   PCOMM_PALM_ADC_VCHG_MV,         // Wall charger voltage at jack
   PCOMM_PALM_ADC_ICHG_MV,         // Voltage across the charger current sense resistor
   PCOMM_PALM_ADC_ICHG_OUT_MV,     // Handset supply voltage (Vdd) determining current
                                   //    leaving the battery
   PCOMM_PALM_ADC_BATT_ID_MV,      // Main battery ID voltage ???
   PCOMM_PALM_ADC_BATT_THERM_DEGC, // Main battery temperature
   PCOMM_PALM_ADC_USB_VBUS_MV,     // USB charger voltage at jack
   PCOMM_PALM_ADC_PMIC_THERM_DEGC, // PMIC die temperature
   PCOMM_PALM_ADC_CHG_THERM,       // Unconnected: Charger transistor temperature
   PCOMM_PALM_ADC_CHG_THERM_DEGC,  // Unconnected: Charger temperature
   PCOMM_PALM_ADC_SDRAM_THERM_DEGC,// ADIE_MUX_1: SDRAM temperature
                                   // ADIE_MUX_2: HDET1: RF power detect (HDET_PCS)
   PCOMM_PALM_ADC_PA_THERM_DEGC,   // ADIE_MUX_3: RF power amplifier thermistor
   PCOMM_PALM_ADC_MSM_THERM_DEGC,  // Unconnected: MSM thermistor
   PCOMM_PALM_ADC_GSM_BANDGAP_VREF_MV,// ADIE_MUX_6: GSM Bandgap reference voltage
   PCOMM_PALM_ADC_RF_HDET,         // HDET read by internal RF Tx ADC
   PCOMM_PALM_ADC_RF_HDET_THERM,   // HDET thermistor read by internal RF Tx ADC
   //     !!!  Following are for ADC's INTERNAL use only  !!!
   //        !!!  Clients of ADC shall NEVER use them  !!!
   PCOMM_PALM_ADC_INTERNAL_USE_CHAN_1,
   PCOMM_PALM_ADC_INTERNAL_USE_CHAN_2,
   PCOMM_PALM_ADC_HS_SWITCH_DETECT,  // Headset Switch Detect   
} pcomm_palm_adc_channel_t;


/* 
 * These are for Palm internal definition of the VREG ID, which is mapped 
 * for something else...i.e MIC
 */
typedef enum
{
	PALM_VREG_ID_START = 100,
	/*
	 * 	enable internal mic bias
	 */
	PALM_VREG_ID_MIC,

	/*
	 * 	read the ADIE control registers
	 */
	PALM_VREG_CODEC_READ,
	/*
	 * 	write to adie control registers
	 */
	PALM_VREG_CODEC_WRITE,
	/*
	 *	read an ADC channel
	 */
	PALM_VREG_ADC_READ,
	PALM_VREG_ID_MAX,
} palm_vreg_id_type; 


typedef enum {
	PALM_HEADSET_UNINITIALIZED = 0,
	PALM_HEADSET_DETECTED = 1,
	PALM_STEREO_HEADSET_DETECTED = 2,
	PALM_HEADSET_NOT_DETECTED = 3,
} palm_headset_status_type;

#endif

/* gpio info for PCOM_RPC_GPIO_TLMM_CONFIG_EX */

#define GPIO_ENABLE	0
#define GPIO_DISABLE	1

#define GPIO_INPUT	0 
#define GPIO_OUTPUT	1 
 
#define GPIO_NO_PULL	0 
#define GPIO_PULL_DOWN	1 
#define GPIO_KEEPER	2 
#define GPIO_PULL_UP	3 
 
#define GPIO_2MA	0 
#define GPIO_4MA	1 
#define GPIO_6MA	2 
#define GPIO_8MA	3 
#define GPIO_10MA	4 
#define GPIO_12MA	5 
#define GPIO_14MA	6 
#define GPIO_16MA	7 

#define ALT_FN_0	0
#define ALT_FN_1	1
#define ALT_FN_2	2
#define ALT_FN_3	3
#define ALT_FN_4	4
#define ALT_FN_5	5
#define ALT_FN_6	6
#define ALT_FN_7	7
 
#define PCOM_GPIO_CFG(gpio, func, dir, pull, drvstr) \
		((((gpio) & 0x3FF) << 4)	| \
		((func) & 0xf)			| \
		(((dir) & 0x1) << 14)		| \
		(((pull) & 0x3) << 15)		| \
		(((drvstr) & 0xF) << 17))

int msm_proc_comm(unsigned cmd, unsigned *data1, unsigned *data2);

#endif
