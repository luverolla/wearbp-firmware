/**
 * @file wearbp.h
 * @brief This file contains necessary defines for the WearBP system
 *
 */

#ifndef INC_WEARBP_H_
#define INC_WEARBP_H_

#include "config.h"
#include "ppg_utils.h"
#include "filter_coefficients.h"

/**
 * @brief Global object for keeping system status
 *
 * One and only one instance of this struct shall exist and its properties
 * shall not be modified.
 *
 * For details about the individual properties, refer to the documentation of
 * the functions in this file.
 */
typedef struct WBP_System {
	uint8_t IsMeasuring;		///< tells when to activate ADC and its timer
	size_t SampleCount; 		///< number of samples acquired from ADC
	uint8_t IsPPGReady; 		///< set once the signal gets all its samples

	uint32_t ADCBuffer[1]; 		///< buffer for ADC acquisition with DMA

	ppg_filter Filter;			///< FIR filter instance
	float FilterState[
	  PPG_SIGLEN +
	  FILT_FIR_NTAPS - 1
    ];							///< buffer storing the FIR filter's state

	float PPGMean;				///< DC component of the raw PPG waveform
	float RawPPG[PPG_SIGLEN]; 	///< buffer for raw PPG waveform
	float FiltPPG[PPG_SIGLEN];	///< buffer for filtered PPG waveform
	float NormPPG[PPG_SIGLEN];	///< buffer for 0-1 normalised PPG waveform

	ppg_fid PPGFiducials;		///< fiducial points of PPG waveform
	float FeatsVector[66];		///< buffer for the feature vector
	float NormFeatures[66];		///< buffer storing normalised features
	float FeatsAccum[
	 PPG_MAX_FID
	];							///< buffer for accumulating feature values

	float LastSBP[
	  CFG_AVGCOUNT
	];							///< last computed values for systolic BP
	float LastDBP[
	  CFG_AVGCOUNT
	];							///< last computed values for diastolic BP
	size_t LastCount;			///< count of computed BP values
	float LastMeanSBP;			///< last mean computed value for systolic BP
	float LastMeanDBP;			///< last mean computed value for diastolic BP

#if CFG_DEBUG == 1
	/**
	 * @brief Transmits the current sample and the two mean values of BP
	 */
	float SerialTxBuffer[3];
#endif
} WBP_System;

/**
 * @brief Initialises the system
 *
 * This function initialises an instance of the ARM CMSIS FIR filter and the
 * necessary peripherals of the STM32 platform, namely the ADC and the TIM2.
 *
 * @param Handle pointer to the global state object
 */
void WBP_Init(WBP_System* Handle);

/**
 * @brief Acquire a sample from ADC
 *
 * This function must be called only when a sample from the ADC is available.
 * It is suggested to set the ADC for operating in DMA mode and to call this
 * function from the HAL_ADC_ConvCpltCallback() callback.
 *
 * @param Handle pointer to the global state object
 */
void WBP_AcquireADC(WBP_System* Handle);

/**
 * @brief Handles button click for manual start of measure
 *
 * If the system is in IDLE status, this function brings it in MEASURE status
 * by resetting the timer.
 *
 * This function must be called within a EXTI callback
 *
 * @param Handle pointer to the global system object
 */
void WBP_HandleButton(WBP_System* Handle);

/**
 * @brief Toggle measurement status based on main timer
 *
 * This functions starts or stops the ADC and its timer whenever an interrupt
 * from the main timer is received, and it must be called inside its callback.
 *
 * @param Handle pointer to the global state object
 */
void WBP_ToggleMeasuremnt(WBP_System* Handle);

/**
 * @brief Process the acquired PPG waveform
 *
 * This function represents the "main loop" of the WearBP firmware. It can be
 * called either from the main while() loop or from a RTOS thread.
 *
 * @param Handle pointer to the global state object
 */
void WBP_ProcessPPG(WBP_System* Handle);

#endif /* INC_WEARBP_H_ */
