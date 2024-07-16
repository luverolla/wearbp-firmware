/**
 * @file config.h
 * @brief This file contains all the configurable parameters for the system
 *
 * Refer and pay attention to the documentation of each parameter before
 * applying any modification to it.
 *
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define CFG_DEBUG 1

/**
 * @brief Window size [seconds] for features extraction from PPG signal
 *
 * Since, for each feature, the median value is extracted in a window, larger
 * windows bring more reliable values, but also increase the waiting time
 * before the first measure is obtained.
 *
 * Tune this parameter and CFG_AVGWIN accordingly to the error variability
 * provided by your statistical or ML model
 */
#define CFG_WINSIZE 5

/**
 * @brief Number of BP values to compute the mean value
 *
 * Bigger values of this parameters will lead to more stable values of BP, but
 * sensibility to variations will decrease.
 *
 * Tune this parameter with care, together with CFG_WINSIZE.
 */
#define CFG_AVGCOUNT 100

/**
 * @brief Handles to Timer and ADC peripherals of STM32 platform
 *
 * Make sure that both perhiperals have been activated in the .ioc file, and
 * that their parameters have been correctly set, namely:
 * - ADC set in DMA mode and ISR activated
 * - Timer prescaler and period set to obtain the chosen sample rate
 *
 */
///@{
#define CFG_DEV_TIM_ADC		(&htim2)
#define CFG_DEV_TIM_MAIN	(&htim3)
#define CFG_DEV_ADC			(&hadc1)
#if CFG_DEBUG == 1
#define CFG_DEV_UART		(&huart2)
#endif
///@}

/**
 * @brief Maximum and minimum admissible limits for blood pressure
 *
 * You can change these values if you plan to use the device for patients that
 * are conditions, for which the range of their blood pressure is limited and
 * well known a-priori. For example: patients with hyper- or hypo-tension.
 *
 * _SBP_VALID() and _DBP_VALID() are macros that encapsulate the upper- and
 * lower- bound comparisons with the limit values.
 */
///@{
#define CFG_MAXLIM_SBP 300
#define CFG_MAXLIM_DBP 200
#define CFG_MINLIM_SBP 40
#define CFG_MINLIM_DBP 20

#define _SBP_VALID(sbp) \
	( ((sbp) > CFG_MINLIM_SBP) && ((sbp) < CFG_MAXLIM_SBP) )

#define _DBP_VALID(dbp) \
	( ((dbp) > CFG_MINLIM_DBP) && ((dbp) < CFG_MAXLIM_DBP) )
///@}

#endif /* INC_CONFIG_H_ */
