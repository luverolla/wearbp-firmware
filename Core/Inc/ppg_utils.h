/**
 * @file ppg_utils.h
 * @brief Utility objects and function to work with PPG signals
 *
 */

#ifndef INC_PPG_UTILS_H_
#define INC_PPG_UTILS_H_

#include "config.h"
#include "dsp/filtering_functions.h"

/**
 * @brief Sampling frequency [Hz] for PPG signal
 *
 * This frequency has been used for the design of the digital filter, as well
 * as the training and validation of the statistical inference model. Moreover,
 * also the clock speed and the parameters of TIM2 have been tuned to match it.
 *
 * For this reason, I suggest you to NOT change this value, and, in case you
 * decide to use other datasets, with signals sampled at different rates, you
 * should consider re-sampling first.
 *
 * But if it's not possible or, for any reason, you need a different frequency,
 * keep in mind that you'll need to re-design the filter and repeat the train
 * and validation of the statistical model with the new value.
 *
 */
#define PPG_FSAMPLE 125

#define PPG_FSAMPLE_MS

/**
 * @brief Length [samples] of PPG signal windows
 *
 * Do not modify directly. Tune the CFG_WINSIZE instead
 *
 */
#define PPG_SIGLEN   (CFG_WINSIZE*PPG_FSAMPLE) // samples

/** @name Maximum and minimum number of fiducial points
 *
 * There should be one set of fiducials for each heart beat.
 *
 * The extreme minimum and maximum measurable values meaningful for humans'
 * heart rate are 45 and 180 bpm, or respectively 0.75 and 3 beats per second.
 *
 * For patients with heart rates out of this range any measurement of blood
 * pressure is considered useless, and if measurements with such values arise,
 * they will be treated as errors. This same criterion has been used during
 * train and validation of the statistical model, therefore, if you decide to
 * change these values you MUST repeat the workflow.
 */
///@{
#define PPG_MIN_FID (int32_t)(0.75*CFG_WINSIZE)
#define PPG_MAX_FID	(3*CFG_WINSIZE)
///@}

/**
 * @brief Shortening alias for the ARM FIR filter structure
 */
typedef arm_fir_instance_f32 ppg_filter;

/**
 * @brief Error values for PPG-related functions
 *
 * Feel free to add more errors in the empty space before PPG_ERMAX. With the
 * current setting, the error variable is packed into a 8-bit unsigned integer.
 * For more details about the individual error, refer to the documentation of
 * the functions in this module.
 */
typedef enum ppg_error {
	PPG_NOERR = 0x00,	///< No error
	PPG_COSIG		,	///< Constant signal, cannot compute fiducials
	PPG_FIDFA		,	///< Fiducial check failed with any possible reason
	PPG_BPOUT		,	///< Estimated BP values out of admissible range

	PPG_ERMAX = 0xFF	///< Reserved, do not use
} ppg_error;

/**
 * @brief Struct containing fiducial points of a PPG signal
 *
 * Each buffer stores the sample (x-axis coordinate) of a signal whose value
 * is a fiducial point and not the value itself
 *
 */
typedef struct ppg_fid {
    uint32_t speaks[PPG_MAX_FID];	///< array of systolic peaks
    size_t n_speaks;				///< number of systolic peaks
    uint32_t dpeaks[PPG_MAX_FID]; 	///< array of diastolic peaks
    size_t n_dpeaks;				///< number of diastolic peaks
    uint32_t valleys[PPG_MAX_FID];	///< array of valleys
    size_t n_valleys;				///< number of valleys
    uint32_t notches[PPG_MAX_FID];	///< array of dicrotic notches
    size_t n_notches;				///< number of dicrotic notches
} ppg_fid;


/**
 * @brief Normalises PPG signal and maps it in the [0,1] range
 *
 * Both input and output pointer MUST refer to memory blocks able to store a
 * number of bytes equal to n*sizeof(float). If the memory areas are shorter,
 * the behaviour of this function cannot be determined.
 *
 * The function is able to detect constant signals, evaluating the result of
 * the equation max(ppg)-min(ppg) == 0. In this case, the error PPG_COSIG is
 * returned and nothing is written to the output pointer.
 *
 * @param ppg the input signal
 * @param n the actual size of the input signal
 * @param dst the pointer where the output signal
 * @returns error code
 */
ppg_error ppg_norm(const float* ppg, size_t n, float* dst);


/**
 * @brief Extract fiducial points from PPG signal
 *
 * The signal is normalised before the necessary computations.
 *
 * Both input and output pointer MUST refer to memory blocks able to store a
 * number of bytes according to the type. If the memory areas are shorter,
 * the behaviour of this function cannot be determined.
 *
 * @param ppg the given PPG signal
 * @param n the size of the given signal
 * @param fid pointer to a struct of fiducial points
 */
void ppg_get_fiducials(const float* ppg, size_t n, ppg_fid* fid);

/**
 * @brief Filters fiducials and retains only good cardiac cycles
 *
 * Analyse the supplied set of fiducials to check that it follows the correct
 * form of a PPG signal. In detail, this function verify that the following
 * conditions hold for each cardiac cycle (from valley v1 to valley v2)
 *
 * - there is a diastolic peak between the v1 and the systolic peak
 * - there are no diastolic peak between a systolic peak and v2
 * - the diastolic peak's height is between 1/10 and 3/4 of the
 *   systolic peak's height
 *
 * If one of these condition doesn't hold, the relative cardiac cycle is
 * removed by modifying the provided fiducial structure. The number of retained
 * cardiac cycle is returned.
 *
 * @param ppg the given PPG signal
 * @param fid the supplied fiducial structure
 * @return the number of retained cardiac cycles
 */
size_t ppg_filt_fiducials(const float* ppg, ppg_fid* fid);

/**
 * @brief Extract features from a PPG signal and a set of fiducials
 *
 * Both input and output pointer MUST refer to memory blocks able to store a
 * number of bytes according to the type. If the memory areas are shorter,
 * the behaviour of this function cannot be determined.
 *
 * This function calls ppg_filt_fiducials() before all necessary computations.
 * If no cardiac cycle has been retained after filtering, this function returns
 * the error code PPG_FIDFA. Otherwise it completes the feature extraction and
 * returns PPG_NOERR.
 *
 *
 * @param ppg the given PPG signal
 * @param n the size of the given signal
 * @param fid the supplied set of fiducials
 * @param feats the vector to write features in
 */
ppg_error ppg_get_features(
	const float* ppg, size_t n, ppg_fid* fid, float* tmp, float* feats
);


#endif /* INC_PPG_UTILS_H_ */
