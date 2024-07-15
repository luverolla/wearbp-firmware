#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "wearbp.h"
#include "filter_coefficients.h"
#include "model_coefs.h"
#include "dsp/statistics_functions.h"

void WBP_Init(WBP_System* Handle) {
	// initialisation of FIR filter state
	memset(
		Handle->FilterState, 0, sizeof(float)*(PPG_SIGLEN+FILT_FIR_NTAPS-1)
	);

	memset(Handle->FeatsVector, 0, sizeof(float)*MDL_NFEATS);
	arm_fir_init_f32(
		&Handle->Filter, FILT_FIR_NTAPS, FILT_FIR_COEFS,
		Handle->FilterState, PPG_SIGLEN
	);

	// starting peripherals
	HAL_TIM_Base_Start_IT(CFG_ADC_TIM);
	HAL_ADC_Start_DMA(CFG_SENSOR_ADC, Handle->ADCBuffer, 1);

	// setting current BPs to invalid values
	Handle->LastMeanSBP = -1.0f;
	Handle->LastMeanDBP = -1.0f;

	// setting counts
	Handle->IsPPGReady = 0;
	Handle->LastCount = 0;
}

void WBP_AcquireADC(WBP_System* Handle) {
	if (Handle->SampleCount < PPG_SIGLEN) {
		Handle->RawPPG[Handle->SampleCount] =
			(Handle->ADCBuffer[0] * 1.0f) * 3.3f / 4095.0f;
		Handle->SampleCount++;
	}
	else {
		Handle->IsPPGReady = 0;
		memmove(
			Handle->RawPPG, &Handle->RawPPG[1], (PPG_SIGLEN-1)*sizeof(float)
		);
		Handle->RawPPG[PPG_SIGLEN-1] =
			(Handle->ADCBuffer[0] * 1.0f) * 3.3f / 4095.0f;
		Handle->IsPPGReady = 1;
	}
}

void WBP_ProcessPPG(WBP_System* Handle) {
	uint8_t err;
	float sbp_pred, dbp_pred;

	if (Handle->IsPPGReady) {
		arm_mean_f32(Handle->RawPPG, PPG_SIGLEN, &Handle->PPGMean);
		arm_fir_f32(
			&Handle->Filter, Handle->RawPPG, Handle->FiltPPG, PPG_SIGLEN
		);
		arm_scale_f32(
			Handle->FiltPPG, 1/(Handle->PPGMean), Handle->FiltPPG, PPG_SIGLEN
		);

		ppg_error norm_outcome = ppg_norm(Handle->FiltPPG, PPG_SIGLEN, Handle->NormPPG);

		if (norm_outcome == PPG_NOERR) {
			ppg_get_fiducials(Handle->NormPPG, PPG_SIGLEN, &Handle->PPGFiducials);
			ppg_filt_fiducials(Handle->FiltPPG, &Handle->PPGFiducials);

			if (Handle->PPGFiducials.n_valleys > 1) {
				err = ppg_get_features(
					Handle->FiltPPG, PPG_SIGLEN, &Handle->PPGFiducials,
					Handle->FeatsAccum, Handle->FeatsVector
				);

				if (!err) {
					// SBP
					arm_sub_f32(
						Handle->FeatsVector, MDL_SBP_OFFSET, Handle->NormFeatures, MDL_NFEATS
					);
					for (size_t i = 0; i < MDL_NFEATS; i++) {
						Handle->NormFeatures[i] /= MDL_SBP_SCALE[i];
					}
					arm_dot_prod_f32(
						Handle->NormFeatures, MDL_SBP_BETA, MDL_NFEATS, &sbp_pred
					);
					sbp_pred += MDL_SBP_BIAS;

					// DBP
					arm_sub_f32(
						Handle->FeatsVector, MDL_DBP_OFFSET, Handle->NormFeatures, MDL_NFEATS
					);
					for (size_t i = 0; i < MDL_NFEATS; i++) {
						Handle->NormFeatures[i] /= MDL_DBP_SCALE[i];
					}
					arm_dot_prod_f32(
						Handle->NormFeatures, MDL_DBP_BETA, MDL_NFEATS, &dbp_pred
					);
					dbp_pred += MDL_DBP_BIAS;

					if (_SBP_VALID(sbp_pred) && _DBP_VALID(dbp_pred)) {
						if (Handle->LastCount < CFG_AVGCOUNT) {
							Handle->LastSBP[Handle->LastCount] = sbp_pred;
							Handle->LastDBP[Handle->LastCount] = dbp_pred;
							Handle->LastCount++;
						} else {
							memmove(
								Handle->LastSBP, &Handle->LastSBP[1],
								(CFG_AVGCOUNT-1)*sizeof(float)
							);
							memmove(
								Handle->LastDBP, &Handle->LastDBP[1],
								(CFG_AVGCOUNT-1)*sizeof(float)
							);
							Handle->LastSBP[CFG_AVGCOUNT-1] = sbp_pred;
							Handle->LastDBP[CFG_AVGCOUNT-1] = dbp_pred;
						}

						arm_mean_f32(
							Handle->LastSBP, Handle->LastCount, &Handle->LastMeanSBP
						);
						arm_mean_f32(
							Handle->LastDBP, Handle->LastCount, &Handle->LastMeanDBP
						);
					}
				}
			}
		}

#if CFG_DEBUG == 1
	//Handle->SerialTxBuffer[0] = Handle->NormPPG[PPG_SIGLEN-1];
	Handle->SerialTxBuffer[0] = Handle->LastMeanSBP;
	Handle->SerialTxBuffer[1] = Handle->LastMeanDBP;
	HAL_UART_Transmit_DMA(
		CFG_DEBUG_UART, (uint8_t*)Handle->SerialTxBuffer, 2*sizeof(float)
	);
#endif
	}
}
