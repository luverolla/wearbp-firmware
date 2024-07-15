#include <stdlib.h>

void vec_ordadd(float* vec, size_t* n, float elem) {
	if (*n == 0) {
		vec[*n] = elem;
	} else {
		size_t i = *n;
		while (i > 0 && vec[i-1] > elem) {
			vec[i] = vec[i-1];
			i--;
		}
		vec[i] = elem;
	}
	*n = *n + 1;
}

float vec_median(const float* vec, size_t n) {
	float res;
	if (n % 2 == 0) {
		res = (vec[n / 2 - 1] + vec[n / 2]) / 2.0;
	} else {
		res = vec[n / 2];
	}
	return res;
}
