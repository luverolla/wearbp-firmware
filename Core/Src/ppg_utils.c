#include <math.h>

#include "dsp/basic_math_functions.h"
#include "dsp/statistics_functions.h"
#include "utils.h"
#include "ppg_utils.h"

/*
 * ============================================================================
 * Private methods
 * ============================================================================
 */

uint32_t pulse_width_k(const float *x, size_t n, float k, float zero) {
	float max_v;
	uint32_t max_i;

	arm_max_f32(x, n, &max_v, &max_i);

	float x_max = max_v - zero;
	float A_k = k * x_max;

	uint8_t start_set = 0;
	uint32_t above_thresh_start = 0;
	uint32_t above_thresh_end = 0;

	for (size_t i = 0; i < n; i++) {
		if (x[i] >= zero + A_k) {
			// start not set: this is first point above threshold
			if (start_set == 0) {
				above_thresh_start = i;
				start_set = 1;
			}

			above_thresh_end = i;
		}
	}

	return (above_thresh_end - above_thresh_start);
}


uint32_t next_fid(
	uint32_t start, uint32_t end, const uint32_t *fid, size_t n_fid
) {
    uint32_t res_fid = 0;
    for (size_t i = 0; i < n_fid; i++) {
        if (fid[i] > start && fid[i] < end) {
            res_fid = fid[i];
            break;
        }
    }

    return res_fid;
}


uint32_t last_fid(
	uint32_t start, uint32_t end, const uint32_t *fid, size_t n_fid,
	size_t* fid_idx
) {
	uint32_t res_fid = 0;
	for (size_t i = 0; i < n_fid; i++) {
		if (fid[i] > start && fid[i] < end) {
			res_fid = fid[i];
			*fid_idx = i;
		}
	}

	return res_fid;
}


uint32_t num_fids(
	uint32_t start, uint32_t end, const uint32_t* fid, size_t n_fid
) {
	uint32_t num_fids = 0;
	for (size_t i = 0; i < n_fid; i++) {
		if (fid[i] > start && fid[i] < end) {
			num_fids++;
		}
	}

	return num_fids;
}


size_t idxof_dicot(const uint32_t* x, size_t n, uint32_t item) {
	uint32_t mid = x[n/2];
	if (item == mid) {
		return n/2;
	}
	else if (item > mid) {
		return idxof_dicot(x+mid, n/2, item);
	}
	else {
		return idxof_dicot(x, n/2, item);
	}
}


size_t remove_fids(uint32_t v1, uint32_t v2, uint32_t* fid, size_t n_fid) {
	uint8_t found = 0;
	size_t first = 0;
	size_t last = 0;
	size_t new_size = n_fid;

	for (size_t i = 0; i < n_fid; i++) {
		if (fid[i] >= v1 && fid[i] <= v2) {
			if (found) {
				last = i;
			} else {
				first = i;
				last = i;
				found = 1;
			}
		}
	}
	if (found) {
		if (last < n_fid-1) {
			memmove(&fid[first], &fid[last+1], sizeof(uint32_t)*(n_fid-(last+1)));
		}
		new_size -= (last - first + 1);
	}

	return new_size;
}

void remove_cardiac_cycle(ppg_fid* fid, size_t cycle_idx) {
	uint32_t v1 = fid->valleys[cycle_idx];
	uint32_t v2 = fid->valleys[cycle_idx+1];

	fid->n_speaks = remove_fids(v1, v2, fid->speaks, fid->n_speaks);
	fid->n_dpeaks = remove_fids(v1, v2, fid->dpeaks, fid->n_dpeaks);
	fid->n_notches = remove_fids(v1, v2, fid->notches, fid->n_notches);
	// will remove v1 and v2
	fid->n_valleys = remove_fids(v1, v2, fid->valleys, fid->n_valleys);

}

/*
 * ============================================================================
 * Public methods
 * ============================================================================
 */

ppg_error ppg_norm(const float* ppg, size_t n, float* dst) {
	ppg_error res = PPG_COSIG;
	float min_v, max_v;
	uint32_t min_idx, max_idx;

	arm_min_f32(ppg, n, &min_v, &min_idx);
	arm_max_f32(ppg, n, &max_v, &max_idx);

	if (max_v > min_v) {
		res = PPG_NOERR;
		for (size_t i = 0; i < n; i++) {
			dst[i] = (ppg[i] - min_v);
		}
		arm_scale_f32(dst, 1/(max_v-min_v), dst, n);
	}

	return res;
}

void ppg_get_fiducials(const float* ppg, size_t n, ppg_fid* fid) {

    // systolic/diastolic peaks
    fid->n_speaks = 0;
    fid->n_dpeaks = 0;
    for (size_t i = 3; i < n; i++) {
        if (
			ppg[i - 2] > ppg[i - 3] && ppg[i - 1] <= ppg[i - 2] &&
			ppg[i - 1] > ppg[i]
        ) {

            if (ppg[i - 2] >= 0.65 && fid->n_speaks < PPG_MAX_FID) {
                fid->speaks[fid->n_speaks] = i - 2;
                fid->n_speaks += 1;
            } else if (ppg[i - 2] >= 0.2 && fid->n_dpeaks < PPG_MAX_FID) {
                fid->dpeaks[fid->n_dpeaks] = i - 2;
                fid->n_dpeaks += 1;
            }
        }
    }

    // valleys/notches
    float vl_v;
    uint32_t vl_idx;
    fid->n_valleys = 0;
    for (size_t i = 0; i < (fid->n_speaks - 1); i++) {
    	if (fid->n_valleys < PPG_MAX_FID) {
			arm_min_f32(
				ppg + fid->speaks[i], fid->speaks[i + 1] - fid->speaks[i],
				&vl_v, &vl_idx
			);
			fid->valleys[fid->n_valleys] = fid->speaks[i] + vl_idx;
			fid->n_valleys += 1;
    	}
    }

    fid->n_notches = 0;
    if (fid->n_dpeaks > 0) {
        for (size_t i = 0; i < (fid->n_speaks - 1); i++) {
            uint32_t dpeak = next_fid(
				fid->speaks[i], fid->speaks[i + 1], fid->dpeaks, fid->n_dpeaks
			);
            if (dpeak > 0 && fid->n_notches < PPG_MAX_FID) {
                float notch_v;
                uint32_t notch_idx;
                arm_min_f32(
					ppg + fid->speaks[i], dpeak - fid->speaks[i], &notch_v,
					&notch_idx
				);
                fid->notches[fid->n_notches] = fid->speaks[i] + notch_idx;
                fid->n_notches += 1;
            }
        }
    }
}

ppg_error ppg_filt_fiducials(const float* ppg, ppg_fid* fid) {
	// index of first valley of cardiac cycle
	size_t v_idx = 0;

	// current number of valleys after check of each cardiac cycle
	size_t curr_n_vls = fid->n_valleys;

	// check each cardiac cycle until there are no more
	while (curr_n_vls > 1 && v_idx < curr_n_vls-1) {
		// current (v1) and next (v2) valley
		uint32_t v1 = fid->valleys[v_idx];
		uint32_t v2 = fid->valleys[v_idx+1];
		// systolic peak between v1 and v2
		uint32_t sp = next_fid(v1, v2, fid->speaks, fid->n_speaks);

		// error: no diastolic peak between systolic peak and next valley
		if (num_fids(sp, v2, fid->dpeaks, fid->n_dpeaks) < 1) {
			remove_cardiac_cycle(fid, v_idx);
			curr_n_vls -= 2;
			continue;
		}

		// error: unwanted peak (resembling a diastolic peak) between a valley
		// 		  and the nearby systolic peak
		if (num_fids(v1, sp, fid->dpeaks, fid->n_dpeaks) > 0) {
			remove_cardiac_cycle(fid, v_idx);
			curr_n_vls -= 2;
			continue;
		}

		// error: diastolic peak taller than 3/4 or shorter than 1/10 of
		//        systolic peak's height
		uint32_t dp = next_fid(v1, v2, fid->dpeaks, fid->n_dpeaks);
		if (
			(ppg[dp] - ppg[v1]) >= 0.75*(ppg[sp] - ppg[v1])
			|| (ppg[dp] - ppg[v1]) <= 0.1*(ppg[sp] - ppg[v1])
		) {
			remove_cardiac_cycle(fid, v_idx);
			curr_n_vls -= 2;
			continue;
		}

		v_idx++;
	}

	// I need cardiac cycles
	return ((curr_n_vls / 2) > 0) ? PPG_NOERR : PPG_FIDFA;
}

void ppg_get_features(
	const float* ppg, size_t n, ppg_fid* fid, float* tmp, float* feats
) {
	size_t filled;
	float val;
	size_t curr_feat = 0;

	feats[2] = 0.0f;   // sex (0=male, 1=female)
	feats[3] = 184.0f; // height [cm]
	feats[4] = 110.0f; // weight [kg]
	feats[0] = 24.0;  // age [years]
	feats[1] = 32.0;  // BMI [kg/m^2]

	// --- TIME FEATURES ---
	// X (systolic peak)
	curr_feat = 5;
	filled = 0;
	for (size_t i = 0; i < fid->n_valleys - 1; i++) {
		uint32_t speak = next_fid(
			fid->valleys[i], fid->valleys[i + 1], fid->speaks, fid->n_speaks
		);
		if (speak > 0) {
			val = ppg[speak] - ppg[fid->valleys[i]];
			vec_ordadd(tmp, &filled, val);
		}
	}
	feats[curr_feat] = vec_median(tmp, filled);


	// Y (diastolic peak)
	curr_feat++;
	filled = 0;
	for (size_t i = 0; i < fid->n_valleys - 1; i++) {
		uint32_t dpeak = next_fid(
			fid->valleys[i], fid->valleys[i + 1], fid->dpeaks, fid->n_dpeaks
		);
		if (dpeak > 0) {
			val = ppg[dpeak] - ppg[fid->valleys[i]];
			vec_ordadd(tmp, &filled, val);
		}
	}
	feats[curr_feat] = vec_median(tmp, filled);

	// Z (notch)
	curr_feat++;
	filled = 0;
	for (size_t i = 0; i < fid->n_valleys - 1; i++) {
		uint32_t notch = next_fid(
			fid->valleys[i], fid->valleys[i + 1], fid->notches, fid->n_notches
		);
		if (notch > 0) {
			val = ppg[notch] - ppg[fid->valleys[i]];
			vec_ordadd(tmp, &filled, val);
		}
	}
	feats[curr_feat] = vec_median(tmp, filled);


	// T1 (systolic peak time)
	curr_feat++;
	filled = 0;
	for (size_t i = 0; i < fid->n_valleys - 1; i++) {
		uint32_t speak = next_fid(
			fid->valleys[i], fid->valleys[i + 1], fid->speaks, fid->n_speaks
		);
		if (speak > 0) {
			val = (speak - fid->valleys[i]) * 1000 / PPG_FSAMPLE;
			vec_ordadd(tmp, &filled, val);
		}
	}
	feats[curr_feat] = vec_median(tmp, filled);


	// T3 (diastolic time)
	curr_feat++;
	filled = 0;
	for (size_t i = 0; i < fid->n_valleys - 1; i++) {
		uint32_t dpeak = next_fid(
			fid->valleys[i], fid->valleys[i + 1], fid->dpeaks, fid->n_dpeaks
		);
		if (dpeak > 0) {
			val = (dpeak - fid->valleys[i]) * 1000 / PPG_FSAMPLE;
			vec_ordadd(tmp, &filled, val);
		}
	}
	feats[curr_feat] = vec_median(tmp, filled);


	// T2 (notch time)
	curr_feat++;
	filled = 0;
	for (size_t i = 0; i < fid->n_valleys - 1; i++) {
		uint32_t notch = next_fid(
			fid->valleys[i], fid->valleys[i + 1], fid->notches, fid->n_notches
		);
		if (notch > 0) {
			val = (notch - fid->valleys[i]) * 1000 / PPG_FSAMPLE;
			vec_ordadd(tmp, &filled, val);
		}
	}
	feats[curr_feat] = vec_median(tmp, filled);


	// DT (DeltaT, distance between systolic and diastolic peak)
	curr_feat++;
	filled = 0;
	for (size_t i = 0; i < fid->n_valleys - 1; i++) {
		uint32_t speak = next_fid(
			fid->valleys[i], fid->valleys[i + 1], fid->speaks, fid->n_speaks
		);
		uint32_t dpeak = next_fid(
			fid->valleys[i], fid->valleys[i + 1], fid->dpeaks, fid->n_dpeaks
		);
		if (speak > 0 && dpeak > speak) {
			val = (dpeak - speak) * 1000 / PPG_FSAMPLE;
			vec_ordadd(tmp, &filled, val);
		}
	}
	feats[curr_feat] = vec_median(tmp, filled);


	// TPI (valley-valley distance)
	curr_feat++;
	filled = 0;
	for (size_t i = 0; i < fid->n_valleys - 1; i++) {
		val = (fid->valleys[i + 1] - fid->valleys[i]) * 1000 / PPG_FSAMPLE;
		vec_ordadd(tmp, &filled, val);
	}
	feats[curr_feat] = vec_median(tmp, filled);


	// TPP (peak-peak distance)
	curr_feat++;
	filled = 0;
	for (size_t i = 0; i < fid->n_speaks - 1; i++) {
		val = (fid->speaks[i + 1] - fid->speaks[i]) * 1000 / PPG_FSAMPLE;
		vec_ordadd(tmp, &filled, val);
	}
	feats[curr_feat] = vec_median(tmp, filled);


	// HR (heart rate)
	curr_feat++;
	// "tmp" already contains peak-peak distance
	// and "filled" contains the number of values
	// we use the values inside for computing heart rate
	for (size_t i = 0; i < filled; i++) {
		tmp[i] = 60000 / tmp[i];
	}
	feats[curr_feat] = vec_median(tmp, filled);


	// HRV (heart rate variability)
	curr_feat++;
	// "tmp" already contains peak-peak distance
	// and "filled" contains the number of values
	// we use the values inside for computing heart rate
	arm_std_f32(tmp, filled, &feats[curr_feat]);


	// IPA (inflection point area)
	curr_feat++;
	filled = 0;
	for (size_t i = 0; i < fid->n_valleys - 1; i++) {
		uint32_t notch = next_fid(
			fid->valleys[i], fid->valleys[i + 1], fid->notches, fid->n_notches
		);
		if (notch > 0) {
			// systolic part (v1:notch)
			float s_area = 0.0f;
			for (size_t j = fid->valleys[i]; j < notch; j++) {
				s_area += fabsf(ppg[j + 1] - ppg[j]);
			}
			// diastolic part (notch:v2)
			float d_area = 0.0f;
			for (size_t j = notch; j < fid->valleys[i + 1]; j++) {
				d_area += fabsf(ppg[j + 1] - ppg[j]);
			}
			val = (s_area / d_area);
			vec_ordadd(tmp, &filled, val);
		}
	}
	feats[curr_feat] = vec_median(tmp, filled);


	// AGI (agumentation index, Y/X)
	curr_feat++;
	// Y is at index 6, X is at index 5
	feats[curr_feat] = feats[6] / feats[5];

	// ALTAGI (alternate agumentation index, (Y-X)/X)
	curr_feat++;
	// Y is at index 6, X is at index 5
	feats[curr_feat] = (feats[5] - feats[6]) / feats[5];

	// SOC (systolic output curve, T1/X) 3/0
	curr_feat++;
	// T1 at index 8, X at index 5
	feats[curr_feat] = feats[8] / feats[5];

	// DDC (diastolic downward curve, Y/(TPI-T2)
	curr_feat++;
	feats[curr_feat] = feats[6] / (feats[12] - feats[10]);

	// T1/TPP, T3/TPP, T2/TPP, DT/TPP
	// TPP at index 8
	// T1,T3,T2,DT at index from 3 to 6 included
	for (size_t j = 8; j <= 11; j++) {
		curr_feat++;
		feats[curr_feat] = feats[j] / feats[13];
	}

	// Z/X
	curr_feat++;
	feats[curr_feat] = feats[7] / feats[5];

	// T2/Z
	curr_feat++;
	feats[curr_feat] = feats[10] / feats[7];

	// T3/Y
	curr_feat++;
	feats[curr_feat] = feats[9] / feats[6];

	// X/(TPI-T1)
	curr_feat++;
	feats[curr_feat] = feats[5] / (feats[12] - feats[8]);

	// Z/(TPI-T2)
	curr_feat++;
	feats[curr_feat] = feats[7] / (feats[12] - feats[10]);

	// pulse widths
	float fractions[3] = {0.25f, 0.5f, 0.75f};
	uint8_t frac_idx = 0;
	// index of first width feature (W25)
	size_t firstW = curr_feat+1;
	while (frac_idx < 3) {
		curr_feat++;
		filled = 0;
		for (size_t i = 0; i < fid->n_valleys - 1; i++) {
			const float *pulse = &ppg[fid->valleys[i]];
			size_t pulse_length = fid->valleys[i + 1] - fid->valleys[i];
			uint32_t pulse_width_val = pulse_width_k(
				pulse, pulse_length, fractions[frac_idx],
				ppg[fid->valleys[i]]
			);

			if (pulse_width_val > 0) {
				val = pulse_width_val * 1000 / PPG_FSAMPLE;
				vec_ordadd(tmp, &filled, val);
			}
		}
		feats[curr_feat] = vec_median(tmp, filled);
		frac_idx++;
	}

	// pulse width ratios
	// index of first pulse width feature (W25)
	for (uint8_t f = 0; f < 3; f++) {
		// ratios from pw/T1 to pw/TPI
		for (uint8_t k = 8; k <= 12; k++) {
			curr_feat++;
			feats[curr_feat] = feats[firstW + f] / feats[k];
		}
	}

	// interactions between demographic and time-domain features
	curr_feat++;
	feats[curr_feat] = feats[3] / feats[11];
	curr_feat++;
	feats[curr_feat] = feats[4] / feats[11];
	curr_feat++;
	feats[curr_feat] = feats[1] / feats[11];

	curr_feat++;
	feats[curr_feat] = feats[3] / feats[8];
	curr_feat++;
	feats[curr_feat] = feats[4] / feats[8];
	curr_feat++;
	feats[curr_feat] = feats[1] / feats[8];

	curr_feat++;
	feats[curr_feat] = feats[3] / feats[10];
	curr_feat++;
	feats[curr_feat] = feats[4] / feats[10];
	curr_feat++;
	feats[curr_feat] = feats[1] / feats[10];

	curr_feat++;
	feats[curr_feat] = feats[3] / feats[9];
	curr_feat++;
	feats[curr_feat] = feats[4] / feats[9];
	curr_feat++;
	feats[curr_feat] = feats[1] / feats[9];

	curr_feat++;
	feats[curr_feat] = feats[3] / feats[12];
	curr_feat++;
	feats[curr_feat] = feats[4] / feats[12];
	curr_feat++;
	feats[curr_feat] = feats[1] / feats[12];

	curr_feat++;
	feats[curr_feat] = feats[3] / feats[13];
	curr_feat++;
	feats[curr_feat] = feats[4] / feats[13];
	curr_feat++;
	feats[curr_feat] = feats[1] / feats[13];
}
