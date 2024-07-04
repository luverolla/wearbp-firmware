/*
 * wavelet.h
 *
 *  Created on: May 10, 2024
 *      Author: Luigi Verolla
 */

#ifndef INC_WAVELET_H_
#define INC_WAVELET_H_

void downsample(const float* x, size_t n, float* y);
void downsample_ip(float* x, size_t n);
void upsample(const float* x, size_t n, float* y);
void convsame(const float* x, size_t n, const float* ker, size_t k, float* y);
void wavedec(const float *x, float *app, float **det, size_t* lens);
void waverec(const float *app, const float **det, const size_t* lens, float *y);


#endif /* INC_WAVELET_H_ */
