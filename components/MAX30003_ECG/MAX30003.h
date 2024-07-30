/**
 * @brief MADE BY TRUONG MINH KHOAb
 * Spirit boi
 * 
 */
#ifndef _MAX30003_H
#define _MAX30003_H
#pragma once
#include "driver/spi_master.h"
#include "esp_bit_defs.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/**********************************************************************************/
/**
 * @name STATUS REGISTER
 * @brief Register Status and its bits
 */
#define REG_STATUS          0x01

#define STATUS_EINT         BIT23 //ECG FIFO Interrupt
#define STATUS_EOVF         BIT22 //ECG FIFO Overflow
#define STATUS_FSINT        BIT21 //ECG Fast Recovery Mode
#define STATUS_DCLOFF_INT   BIT20 //DC Lead-Off Dectect Interrupt
#define STATUS_LONINT       BIT11 //Ultra-Low Power Leads On Dectection Interrupt
#define STATUS_RRINT        BIT10 //R to R Detector R Event Interrupt
#define STATUS_SAMP         BIT9  //R to R Detector R Event Interrupt
#define STATUS_PLLINT       BIT8  //PLL Unlocked Interrupt

#define STATUS_LDOFF_PH     BIT3  //ECGP is above the high threshold
#define STATUS_LDOFF_PL     BIT2  //ECGP is below the low threshold
#define STATUS_LDOFF_NH     BIT1  //ECGN is above the high threshold
#define STATUS_LDOFF_NL     BIT0  //ECGN is below the low threshold
/**********************************************************************************/
/**
 * @name ENABLE INTERRUPT REGISTER
 * @brief Register that govern the operation of the INTB output and INT2B output
 */
#define REG_ENINTB       0x02
#define REG_ENINT2B      0x03

#define EN_EINT         BIT23
#define EN_EOVF         BIT22
#define EN_FSTINT       BIT21 
#define EN_DCLOFFINT    BIT20
#define EN_LONINT       BIT11
#define EN_RRINT        BIT10
#define EN_SAMP         BIT9
#define EN_PPLINT       BIT8

#define EN_INTB_DISABLED    (BIT1 & BIT0) //00 Disabled
#define EN_INTB_CMOS        BIT0            //01 CMOS Driver
#define EN_INTB_OD          BIT1            //10 Open-Drain NMOS Driver
#define EN_INTB_OD_PU       BIT1 | BIT0     //11 Open-Drain NMOS Driver Internal 125kOhm pullup resistance
/**********************************************************************************/
/**
 * @name MANAGE INTERRUPT REGISTER 
 * @brief Manage operation register of the configurable interrupt 
 * bits in response to ECG FIFO conditions
 */
#define REG_MNGR_INT    0x04

/**
 * @brief ECG FIFO Interrupt Threshold (issues EINT based on number of unread
 * FIFO records)
 * 00000 to 11111 = 1 to 32, respectively (i.e. EFIT[4:0]+1 unread records)
 * EFIT[4:0] value will be set inside the code
 */
#define MNGR_INT_EFIT_POS   19 // EFIT position

/**
 * @brief FAST MODE Interrupt Clear
 * 0 = FSTINT remains active until the FAST mode is disengaged (manually or
 * automatically), then held until cleared by STATUS read back (32nd SCLK).
 * 1 = FSTINT remains active until cleared by STATUS read back (32nd SCLK),
 * even if the MAX30003 remains in FAST recovery mode. Once cleared,
 * FSTINT will not be re-asserted until FAST mode is exited and re-entered,
 * either manually or automatically
 */
#define MNGR_INT_CLR_FAST   BIT6


/** @brief RTOR R Detect Interrupt (RRINT) Clear Behavior:
 *  CLR_RRINT[1:0]:
 *  00 = Clear RRINT on STATUS Register Read Back
 *  01 = Clear RRINT on RTOR Register Read Back
 *  10 = Self-Clear RRINT after one ECG data rate cycle, approximately 2ms to 8ms
 *  11 = Reserved. Do not use
 */
#define MNGR_INT_CLR_RRINT_STATUS    (BIT4 & BIT5)
#define MNGR_INT_CLR_RRINT_RTOR    BIT4
#define MNGR_INT_CLR_RRINT_SELF    BIT5

/**
 * @brief Sample Synchronization Pulse (SAMP) Clear
 * 0 = Clear SAMP on STATUS Register Read Back (recommended for debug/evaluation only).
 * 1 = Self-clear SAMP after approximately one-fourth of one data rate cycle.
 */
#define MNGR_INT_CLR_SAMP   BIT2

/** 
 * @brief Sample Synchronization Pulse (SAMP) Frequency
 * 00 = issued every sample instant
 * 01 = issued every 2nd sample instant
 * 10 = issued every 4th sample instant
 * 11 = issued every 16th sample instant
 */
#define MNGR_INT_SAMP_IT_1      0
#define MNGR_INT_SAMP_IT_2      1
#define MNGR_INT_SAMP_IT_4      2
#define MNGR_INT_SAMP_IT_16     3
/**********************************************************************************/
/** 
 * @name MANAGE GENERAL/DYNAMIC MODE
 * @brief Manages the settings of any general/dynamic modes within the device
 */
#define REG_MNGR_DYN 0x05

/**
 * @brief ECG Channel Fast Recovery Mode Selection (ECG High Pass Filter Bypass):
 * 00 = Normal Mode (Fast Recovery Mode Disabled)
 * 01 = Manual Fast Recovery Mode Enable (remains active until disabled)
 * 10 = Automatic Fast Recovery Mode Enable (Fast Recovery automatically activated when/while 
 * ECG outputs are saturated, using FAST_TH)
 * 11 = Reserved. Do not use
 */
#define MNGR_DYN_FAST_NORMAL (BIT22 & BIT23)
#define MNGR_DYN_FAST_MANUAL BIT22
#define MNGR_DYN_FAST_AUTO BIT23

/**
 * @brief Automatic Fast Recovery Threshold:
 * If FAST[1:0] = 10 and the output of an ECG measurement exceeds the symmetric
 * thresholds defined by 2048*FAST_TH for more than 125ms, the Fast Recovery
 * mode will be automatically engaged and remain active for 500ms.
 * For example, the default value (FAST_TH = 0x3F) corresponds to an ECG output
 * upper threshold of 0x1F800, and an ECG output lower threshold of 0x20800.
 * FAST_TH value will be set inside the code
 */
#define MNGR_DYN_FAST_TH_POS 16
/**********************************************************************************/
/** 
 * @name SOFTWARE RESET REGISTER
 * @name SYNCHRONIZE RESET REGISTER
 * @name FIFO RESET REGISTER
 */
#define REG_SW_RST 0x08

#define REG_SYNCH_RST 0x09

#define REG_FIFO_RST 0x0A
/**********************************************************************************/
/**
 * @name INFOMATION, REVISION ID REGISTER
 * @brief Revision ID readback (read-only)
 */
#define REG_INFO        0x0F
#define RevisionID      (0b1111 << 16)
/**********************************************************************************/
/**
 * @name CONFIG GENERAL
 * @brief  which governs general settings, most significantly the master 
 * clock rate for all internal timing operations
 */
#define REG_GEN 0x10

/**
 * @brief Ultra-Low Power Lead-On Detection Enable
 * ULP mode is only active when the ECG channel is powered down/disabled.
 */
#define GEN_EN_ULP_LON BIT22 

/**
 * @brief Master Clock Frequency
 * Selects the Master Clock Frequency (FMSTR), and Timing Resolution (TRES), 
 * which also determines the ECG and CAL timing characteristics.These are generated 
 * from FCLK, which is always 32.768Khz
 * 
 */
#define GEN_FMSTR_32768_512HZ (BIT21 & BIT20)
#define GEN_FMSTR_32000_500HZ  BIT20
#define GEN_FMSTR_32000_200HZ  BIT21
#define GEN_FMSTR_31968_199HZ  BIT21 | BIT20


#define GEN_EN_ECG BIT19 // ECG Channel Enable

/**
 * @brief DC Lead-Off Detection Enable
 * DC Method, requires active selected channel, enables DCLOFF interrupt 
 * and status bit behavior.
 * Uses current sources and comparator thresholds set below.
 * 
 */
#define GEN_DCLOFF_EN BIT12 

/**
 * @brief DC Lead-Off Current Polarity (if current sources are enabled/connected)
 * 0 = ECGP - Pullup ECGN – Pulldown 
 * 1 = ECGP - Pulldown ECGN – Pullup
 * 
 */
#define GEN_IPOL_ECGP_PD_ECGN_PU BIT11
#define GEN_IPOL_ECGP_PU_ECGN_PD (0<<11)

/**
 * @brief DC Lead-Off Current Magnitude Selection
 * 000 = 0nA (Disable and Disconnect Current Sources)
 * 001 = 5nA
 * 010 = 10nA
 * 011 = 20nA
 * 100 = 50nA
 * 101 = 100nA
 */
#define GEN_IMAG_0nA 0
#define GEN_IMAG_5nA BIT8
#define GEN_IMAG_10nA BIT9
#define GEN_IMAG_20nA (BIT8 | BIT9)
#define GEN_IMAG_50nA BIT10
#define GEN_IMAG_100nA (BIT8 | BIT10)

/**
 * @brief DC Lead-Off Voltage Threshold Selection
 * 00 = VMID ± 300mV
 * 01 = VMID ± 400mV
 * 10 = VMID ± 450mV
 * 11 = VMID ± 500mV
 */
#define GEN_DCLOFF_VTH_300mV (BIT6&BIT7)
#define GEN_DCLOFF_VTH_400mV BIT6
#define GEN_DCLOFF_VTH_450mV BIT7
#define GEN_DCLOFF_VTH_500mV (BIT6|BIT7)

/**
 * @brief Enable and Select Resistive Lead Bias Mode
 * 00 = Resistive Bias disabled
 * 01 = ECG Resistive Bias enabled if EN_ECG is also enabled
 * If EN_ECG is not asserted at the same time as prior to EN_RBIAS[1:0] being set to
 * 01, then EN_RBIAS[1:0] will remain set to 00
 */
#define GEN_EN_RBIAS BIT4


/**
 * @brief Resistive Bias Mode Value Selection
 * 00 = RBIAS = 50MΩ
 * 01 = RBIAS = 100MΩ
 * 10 = RBIAS = 200MΩ
 * 11 = Reserved. Do not use.
 * 
 */
#define GEN_RBIASV_50MOHM  (BIT3 & BIT2)
#define GEN_RBIASV_100MOHM  BIT2
#define GEN_RBIASV_200MOHM  BIT3

/**
 * @brief Enables Resistive Bias on Positive Input
 * 0 = ECGP is not resistively connected to VMID
 * 1 = ECGP is connected to VMID through a resistor (selected by RBIASV).
 * 
 */
#define GEN_RBIASP_EN  BIT1

/**
 * @brief Enables Resistive Bias on Negative Input
 * 0 = ECGN is not resistively connected to VMID
 * 1 = ECGN is connected to VMID through a resistor (selected by RBIASV).
 * 
 */
#define GEN_RBIASN_EN  BIT0




/**********************************************************************************/
/** 
 * @name CONFIG CALIBRATION
 * @brief configures the operation, settings, and function of the Internal Calibration Voltage Sources
 * (VCALP and VCALN) The output of the voltage sources can be routed to the ECG inputs through the channel
 * input MUXes to facilitate end-to-end testing operations
 */
#define REG_CAL 0x12

/**
 * @brief Calibration Source (VCALP and VCALN) Enable
 * 0 = Calibration sources and modes disabled
 * 1 = Calibration sources and modes enabled
 */
#define CAL_EN_VCAL BIT22 //Calibration Source (VCALP and VCALN) Enable

/**
 * @brief Calibration Source Mode Selection
 * 0 = Unipolar, sources swing between VMID ± VMAG and VMID
 * 1 = Bipolar, sources swing between VMID + VMAG and VMID - VMAG
 */
#define CAL_VMODE_UNIPOLAR  (0<<21) //Calibration Source Mode Selection
#define CAL_VMODE_BIPOLAR   BIT21 //Calibration Source Mode Selection

/**
 * @brief Calibration Source Magnitude Selection (VMAG)
 * 0 = 0.25mV
 * 1 = 0.50mV
 */
#define CAL_VMAG_025mV (0<<20)
#define CAL_VMAG_050mV BIT20

/**
 * Calibration Source Frequency Selection (FCAL)
 * 000 = FMSTR/128 (Approximately 256Hz)
 * 001 = FMSTR /512 (Approximately 64Hz)
 * 010 = FMSTR /2048 (Approximately 16Hz)
 * 011 = FMSTR /8192 (Approximately 4Hz)
 * 100 = FMSTR /215 (Approximately 1Hz)
 * 101 = FMSTR /217 (Approximately 1/4Hz)
 * 110 = FMSTR /219 (Approximately 1/16Hz)
 * 111 = FMSTR /221 (Approximately 1/64Hz)
 * Actual frequencies are determined by FMSTR selection (see GEN for
 * details), approximate frequencies are based on a 32768Hz clock (FMSTR[2:0] = 000).
 * TCAL = 1/FCAL.
 */
#define CAL_FCAL_207    (BIT12 & BIT13 & BIT14)
#define CAL_FCAL_209    BIT12
#define CAL_FCAL_211   BIT13  
#define CAL_FCAL_213   (BIT12 | BIT13)
#define CAL_FCAL_215    BIT14
#define CAL_FCAL_217    (BIT12 | BIT14)
#define CAL_FCAL_219    (BIT13 | BIT14)
#define CAL_FCAL_221    (BIT12 | BIT13 | BIT14)
/**
 * @brief Calibration Source Duty Cycle Mode Selection
 * 0 = Use CAL_THIGH to select time high for VCALP and VCALN
 * 1 = THIGH = 50% (CAL_THIGH[10:0] are ignored)
 */
#define CAL_FIFTY      BIT11 

/**
 * @brief Calibration Source Time High Selection
 * THIGH = THIGH[10:0] x CAL_RES
 * CAL_RES is determined by FMSTR selection (see GEN for details)
 * if FMSTR[2:0] = 000,CAL_RES = 30.52µs.
 * THIGH value will be set inside the code
 */
#define CAL_THIGH       0

/**********************************************************************************/
/** 
 * @name CONFIG EMUX
 * @brief EMUX is a read/write register which configures the operation, settings, and 
 * functionality of the Input Multiplexer associated with the ECG channel
 */
#define REG_EMUX 0x14

/**
 * @brief ECG Input Polarity Selection
 * 0 = Non-inverted
 * 1 = Inverted
 */
#define EMUX_POL_INVERTED   BIT23 //ECG Input Polarity Selection (1 = Inverted) 

/**
 * @brief Open the ECGP Input Switch (most often used for testing and calibration studies)
 * 0 = ECGP is internally connected to the ECG AFE Channel
 * 1 = ECGP is internally isolated from the ECG AFE Channel
 */
#define EMUX_OPENP BIT21 
#define EMUX_OPENN BIT20 

/**
 * @brief ECGN Calibration Selection
 * 00 = No calibration signal applied
 * 01 = Input is connected to VMID
 * 10 = Input is connected to VCALP (only available if CAL_EN_VCAL = 1)
 * 11 = Input is connected to VCALN (only available if CAL_EN_VCAL = 1)
 * ECG calib select position, shift left EMUX_CALP_SEL_xx to valid 
 * ECGP position or ECGN position (EMUX_CAL_SEL_xx << EMUX_CAL_SEL_POS_ECGN)
 */
#define EMUX_CALP_SEL_NO_CAL   (BIT18 & BIT19) 
#define EMUX_CALP_SEL_VMID     BIT18 
#define EMUX_CALP_SEL_VCALP    BIT19 
#define EMUX_CALP_SEL_VCALN    (BIT18 | BIT19)  

#define EMUX_CALN_SEL_NO_CAL   (BIT17 & BIT16) 
#define EMUX_CALN_SEL_VMID     BIT16 
#define EMUX_CALN_SEL_VCALP    BIT17 
#define EMUX_CALN_SEL_VCALN    (BIT17 | BIT16)
/**********************************************************************************/
/** 
 * @name CONFIG ECG
 * @brief configures the operation, settings, and functionality of the ECG channel.
 * Anytime a change to ECG is made, there may be discontinuities in the ECG record and possibly 
 * changes to thesize of the time steps recorded in the ECG FIFO
 * 
 */
#define REG_ECG        0x15

/**
 * @brief ECG Data Rate (also dependent on FMSTR selection)
 * FMSTR = 00: fMSTR = 32768Hz, tRES = 15.26µs (512Hz ECG progressions)
 * 00 = 512sps
 * 01 = 256sps
 * 10 = 128sps
 * FMSTR = 01: fMSTR = 32000Hz, tRES = 15.63µs (500Hz ECG progressions)
 * 00 = 500sps
 * 01 = 250sps
 * 10 = 125sps
 * FMSTR = 10: fMSTR = 32000Hz, tRES = 15.63µs (200Hz ECG progressions)
 * 10 = 200sps
 * FMSTR = 11: fMSTR = 31968Hz, tRES = 15.64µs (199.8Hz ECG progressions)
 * 10 = 199.8sps
 */
#define ECG_RATE_1     BIT23
#define ECG_RATE_0     BIT22

#define ECG_RATE_512     (BIT22 & BIT23)
#define ECG_RATE_256     BIT22
#define ECG_RATE_128     BIT23

/**
 * @brief ECG Channel Gain Setting
 * 00 = 20V/V
 * 01 = 40V/V
 * 10 = 80V/V
 * 11 = 160V/V
 */
#define ECG_GAIN_20V   (BIT17 & BIT16)
#define ECG_GAIN_40V   BIT16
#define ECG_GAIN_80V   BIT17
#define ECG_GAIN_160V  (BIT17 | BIT16)

/**
 * @brief ECG Channel Digital High-Pass Filter Cutoff Frequency
 * 
 */
#define ECG_DHPF_BYPASS     0<<14
#define ECG_DHPF_05HZ       BIT14 // enable 0.5Hz High-Pass Filter through ECG Channel

/**
 * ECG Channel Digital Low-Pass Filter Cutoff Frequency
 * 00 = Bypass (Decimation only, no FIR filter applied)
 * 01 = approximately 40Hz (Except for 125 and 128sps settings) Note: See Table 33.
 * 10 = approximately 100Hz (Available for 512, 256, 500, and 250sps ECG Rate selections only)
 * 11 = approximately 150Hz (Available for 512 and 500sps ECG Rate selections only)
 */
#define ECG_DLPF_BYPASS (BIT13 & BIT12) 
#define ECG_DLPF_40HZ   BIT12
#define ECG_DLPF_100HZ  BIT13
#define ECG_DLPF_150HZ  (BIT13 | BIT12) 
/**********************************************************************************/
/**
 * 
 * @brief: RTOR is a two-part read/write register that configures the operation, 
 * settings, and function of the RTOR heart rate detection block. The first register contains 
 * algorithmic voltage gain and threshold parameters, the second contains algorithmic timing parameters.
 * 
 */
#define REG_RTOR1  0x1D
#define REG_RTOR2  0x1E


/**
 * @brief This is the width of the averaging window, which adjusts the algorithm sensitivity to
 * the width of the QRS complex.
 * R to R Window Averaging (Window Width = RTOR_WNDW[3:0]*8ms)
 * 0000 = 6
 * 0001 = 8
 * 0010 = 10
 * 0011 = 12 (default = 96ms)
 * 0100 = 14
 * 0101 = 16
 * 0110 = 18
 * 0111 = 20
 * 1000 = 22
 * 1001 = 24
 * 1010 = 26
 * 1011 = 28
 */
#define RTOR1_WNDW_6     (0 << 20)
#define RTOR1_WNDW_8     (1 << 20)
#define RTOR1_WNDW_10    (2 << 20)
#define RTOR1_WNDW_12    (3 << 20)
#define RTOR1_WNDW_14    (4 << 20)
#define RTOR1_WNDW_16    (5 << 20)
#define RTOR1_WNDW_18    (6 << 20)
#define RTOR1_WNDW_20    (7 << 20)
#define RTOR1_WNDW_22    (8 << 20)
#define RTOR1_WNDW_24    (9 << 20)
#define RTOR1_WNDW_26    (10 << 20)
#define RTOR1_WNDW_28    (11 << 20)

/**
 * @brief R to R Gain (where Gain = 2^GAIN[3:0], plus an auto-scale option). This is used to maximize 
 * the dynamic range of the algorithm.
 * In Auto-Scale mode, the initial gain is set to 64.
 * 0000 = 1, 1000 = 256
 * 0001 = 2, 1001 = 512
 * 0010 = 4, 1010 = 1024
 * 0011 = 8, 1011 = 2048
 * 0100 = 16, 1100 = 4096
 * 0101 = 32, 1101 = 8192
 * 0110 = 64, 1110 = 16384
 * 0111 = 128, 1111 = Auto-Scale (default)
 */
#define RTOR1_GAIN_1        (0  << 16)
#define RTOR1_GAIN_2        (1  << 16)
#define RTOR1_GAIN_4        (2  << 16)
#define RTOR1_GAIN_8        (3  << 16)
#define RTOR1_GAIN_16       (4  << 16)
#define RTOR1_GAIN_32       (5  << 16)
#define RTOR1_GAIN_64       (6  << 16)
#define RTOR1_GAIN_128      (7  << 16)
#define RTOR1_GAIN_256      (8  << 16)
#define RTOR1_GAIN_512      (9  << 16)
#define RTOR1_GAIN_1024     (10  << 16)
#define RTOR1_GAIN_2048     (11  << 16)
#define RTOR1_GAIN_4096     (12  << 16)
#define RTOR1_GAIN_8192     (13  << 16)
#define RTOR1_GAIN_16384    (14  << 16)
#define RTOR1_GAIN_AUTO     (15  << 16)


/**
 * @brief ECG RTOR Detection Enable
 * 0 = RTOR Detection disabled
 * 1 = RTOR Detection enabled if EN_ECG is also enabled
 */
#define RTOR1_EN_RTOR        BIT15 

/**
 * @brief R to R Peak Averaging Weight Factor
 * Peak_Average(n) = [Peak(n) + (RTOR_PAVG-1) x Peak_Average(n-1)] / RTOR_PAVG
 * 00 = 2
 * 01 = 4
 * 10 = 8 (default)
 * 11 = 16
 * Peak_Average(n) = [Peak(n) + (RTOR_PAVG-1) x Peak_Average(n-1)] / RTOR_PAVG
 */
#define RTOR1_PAVG_2    (BIT12 & BIT13)
#define RTOR1_PAVG_4    BIT12
#define RTOR1_PAVG_8    BIT13
#define RTOR1_PAVG_16   (BIT12 | BIT13) 

/**
 * @brief R to R Peak Threshold Scaling Factor
 * This is the fraction of the Peak Average value used in the Threshold computation.
 * Values of 1/16 to 16/16 are selected directly by (RTOR_PTSF[3:0]+1)/16, default is 4/16
 * PTFS will be set inside the code
 * Shift left value with RTOR_PTFS_POS position
 */
#define RTOR1_PTSF_POS    8

/**
 * @brief R to R Minimum Hold Off
 * This sets the absolute minimum interval used for the 
 * static portion of the Hold Off criteria.
 * Values of 0 to 63 are supported, default is 32
 * tHOLD_OFF_MIN = HOFF[5:0] * tRTOR, where tRTOR is ~8ms, as determined by
 * FMSTR[1:0] in the GEN register (representing approximately ¼ second).
 * The R to R Hold Off qualification interval is tHold_Off = MAX(tHold_Off_Min, tHold_Off_Dyn)
 * Shift left value with RTOR2_HOFF_POS position
 */
#define RTOR2_HOFF_POS 16

/**
 * @brief R to R Interval Averaging Weight Factor
 * This is the weighting factor for the current RtoR interval observation vs. the past interval
 * observations when determining dynamic holdoff criteria. Lower numbers weight current
 * intervals more heavily.
 * 00 = 2
 * 01 = 4
 * 10 = 8 (default)
 * 11 = 16
 * Interval_Average(n) = [Interval(n) + (RAVG-1) x Interval_Average(n-1)] / RAVG
 */
#define RTOR2_RAVG_2 (BIT12 & BIT13)
#define RTOR2_RAVG_4 BIT12
#define RTOR2_RAVG_8 BIT13
#define RTOR2_RAVG_16 (BIT12 | BIT13)


/**
 * @brief R to R Interval Hold Off Scaling Factor
 * This is the fraction of the RtoR average interval used for the dynamic portion of the holdoff
 * criteria (tHOLD_OFFDYN).
 * Values of 0/8 to 7/8 are selected directly by RTOR_RHSF[3:0]/8, default is 4/8.
 * If 000 (0/8) is selected, then no dynamic factor is used and the holdoff criteria is
 * determined by HOFF[5:0] only 
 * Shift left value with RTOR2_RHSF_POS position
 */
#define RTOR2_RHSF_POS 8

/**********************************************************************************/
#define REG_ECG_FIFO_BURST   0x20
#define REG_ECG_FIFO_NORMAL  0x21
#define REG_RTOR_INTERVAL    0x25

#define ETAG_VALID      0
#define ETAG_FAST       0b001
#define ETAG_VALID_EOF  0b010
#define ETAG_FAST_EOF   0b011
#define ETAG_EMPTY      0b110
#define ETAG_OVERFLOW   0b111
#define ETAG_MASK       0b111
/**********************************************************************************/
typedef union MAX30003_EN_INTB2B_REG_t{
    uint8_t REG_INTB;
    uint8_t REG_INT2B;
}EN_INTB2B_REG_t;
typedef struct MAX30003_EN_INT_t{
    unsigned int  E_INT;
    unsigned int  E_OVF;
    unsigned int  E_FS;
    unsigned int  E_DCOFF;
    unsigned int  E_LON;
    unsigned int  E_RR;
    unsigned int  E_SAMP;
    unsigned int  E_PLL;
    unsigned int  E_INT_TYPE;
    EN_INTB2B_REG_t REG;
}EN_INT_t;
typedef struct MAX30003_MNGR_INT_t{
    unsigned int EFIT;
    unsigned int CLR_FAST;
    unsigned int CLR_RRINT;
    unsigned int CLR_SAMP;
    unsigned int SAMP_IT;
    uint8_t REG;
}MNGR_INT_t;
typedef struct MAX30003_MNGR_DYN_t{
    unsigned int FAST;
    unsigned int FAST_TH;
    uint8_t REG;
}MNGR_DYN_t;
typedef struct MAX30003_GEN_t{
    unsigned int ULP_LON;
    unsigned int FMSTR;
    unsigned int ECG;
    unsigned int DCLOFF;
    unsigned int IPOL;
    unsigned int IMAG;
    unsigned int VTH;
    unsigned int EN_RBIAS;
    unsigned int RBIASV;
    unsigned int RBIASP;
    unsigned int RBIASN;
    uint8_t REG;
}GEN_t;
typedef struct MAX30003_CAL_t{
    unsigned int VCAL;
    unsigned int VMODE;
    unsigned int VMAG;
    unsigned int FCAL;
    unsigned int FIFTY;
    unsigned int THIGH; 
    uint8_t REG;
}CAL_t;
typedef struct MAX30003_EMUX_t{
    unsigned int POL;
    unsigned int OPENP;
    unsigned int OPENN;
    unsigned int CALP;
    unsigned int CALN;
    uint8_t REG;
}EMUX_t;
typedef struct MAX30003_ECG_t{
    unsigned int RATE;
    unsigned int GAIN;
    unsigned int DHPF;
    unsigned int DLPF;
    uint8_t REG;
}ECG_t;

typedef union MAX30003_RTOR_REG_t{
    uint8_t RTOR1;
    uint8_t RTOR2;
}REG_t;
typedef struct MAX30003_RTOR_t{
    unsigned int WNDW;
    unsigned int GAIN;
    unsigned int EN;
    unsigned int PAVG;
    unsigned int PTSF;
    unsigned int HOFF;
    unsigned int RAVG;
    unsigned int RHSF;
    REG_t REG;
}RTOR_t;
typedef struct MAX30003_config_register_t{
     MNGR_DYN_t* DYN;
     MNGR_INT_t* MNGR_INT;
     GEN_t* GEN;
     CAL_t* CAL;
     EMUX_t* EMUX;
     ECG_t* ECG;
     RTOR_t* RTOR;
     EN_INT_t* EN_INT;
}MAX30003_config_register_t;

typedef struct MAX30003_config_register_t_2{
     MNGR_DYN_t DYN;
     MNGR_INT_t MNGR_INT;
     GEN_t GEN;
     CAL_t CAL;
     EMUX_t EMUX;
     ECG_t ECG;
     RTOR_t RTOR;
     EN_INT_t EN_INT;
}MAX30003_config_register_t_2;

typedef struct MAX30003_status_t{
    bool  EINT;
    bool  EOVF;
    bool  FS;
    bool  DCOFF;
    bool  LONINT;
    bool  RR;
    bool  SAMP;
    bool  PLL;
}MAX30003_status_t;
typedef struct MAX30003_config_pin_t{
    spi_host_device_t host; ///< The SPI host used, set before calling `MAX30003_init()`
    gpio_num_t cs_io;       ///< CS gpio number, set before calling `MAX30003_init()`
    gpio_num_t miso_io;     ///< MISO gpio number, set before calling `MAX30003_init()`
    gpio_num_t intb;        ///< INTB pin of MAX30003
    gpio_num_t int2b;       ///< INT2B pin of MAX30003
}MAX30003_config_pin_t;


typedef struct MAX30003_context_t* MAX30003_handle_t;
/**********************************************************************************/
/**
 * @brief Initialize the hardware
 *
 * @param config Configuration of the MAX30003
 * @param out_handle Output context of MAX30003 communication
 * @return
 *  - ESP_OK: on success
 *  - ESP_ERR_INVALID_ARG: If the configuration in the context is incorrect.
 *  - ESP_ERR_NO_MEM: if semaphore create failed.
 *  - or other return value from `spi_bus_add_device()` or `gpio_isr_handler_add()`.
 */
esp_err_t MAX30003_init(const MAX30003_config_pin_t *cfg, MAX30003_handle_t *out_handle);
/**********************************************************************************/
/**
 * @brief Read a register 24 bits from the MAX30003.
 *
 * @param handle Context of MAX30003 communication.
 * @param reg      register to read.
 * @param out_data  Buffer to output 24 bits the read data.
 * @return return value from `spi_device_get_trans_result()`.
 */
esp_err_t MAX30003_read(MAX30003_handle_t handle,uint8_t reg, unsigned int *out_data);
/**********************************************************************************/
/**
 * @brief Write 3 bytes(3,2,1) into the MAX30003
 *
 * @param handle Context of MAX30003 communication.
 * @param reg  register to write.
 * @param in_data  buffer to store 24 bits command of register to write data, 
 * using OR operation to set command
 * 
 *  @return . 
 *  - ESP_OK: on success
 *  - ESP_ERR_TIMEOUT: if the EEPROM is not able to be ready before the time in the spec. This may mean that the connection is not correct.
 *  - or return value from `spi_device_acquire_bus()` `spi_device_polling_transmit()`.
 */
esp_err_t MAX30003_write(MAX30003_handle_t handle,uint8_t reg, unsigned int in_data);
/**********************************************************************************/
/**
 * @brief Check if MAX30003 present on SPI bus
 *
 * @param handle Context of MAX30003 communication.
 *  @return . 
 *  - ESP_OK: on success
 *  - ESP_ERR_INVALID_RESPONSE: Received data from MAX30003 but nibble bits[20:23] is not 0101
 *  - ESP_ERR_TIMEOUT: no respond from MAX30003 
 */
esp_err_t MAX30003_get_info(MAX30003_handle_t handle);
/**********************************************************************************/
/**
 * @brief Read ECG data on normal mode
 *
 * @param handle Context of MAX30003 communication.
 */
esp_err_t MAX30003_read_FIFO_normal(MAX30003_handle_t handle, int16_t *ECG_Sample, uint8_t *Idx);
/**********************************************************************************/
/**
 * @brief Read RTOR 
 *
 * @param handle Context of MAX30003 communication.
 * @param RTOR address that contain value of RTOR
 */
esp_err_t MAX30003_read_RRHR(MAX30003_handle_t handle, uint8_t *hr, unsigned int *RR);

/**********************************************************************************/
/**
 * @brief check ETAG status
 *
 * @param ECG_Data: get value by reading register 0x21 or 0x20.
 */
void MAX30003_check_ETAG(unsigned int ECG_Data);
/**********************************************************************************/
/**
 * @brief set value of register and read back and check if write and read have the same value
 *
 * @param handle Context of MAX30003 communictypation.
 * @param value value to send to MAX30003 and read back to check if whether mismatch read and write
 */
esp_err_t MAX30003_set_get_register(MAX30003_handle_t handle,unsigned int reg,unsigned int value,char *NAME_REG);
/**********************************************************************************/
/**
 * @brief config register of MAX30003
 *
 * @param cfgreg struct pointer which members are pointers to register MAX30003
 * @param handle Context of MAX30003 communication.
 * 
 */
esp_err_t MAX30003_conf_reg(MAX30003_handle_t handle, MAX30003_config_register_t *cfgreg);

void MAX30003_reset(MAX30003_handle_t handle);
#endif
