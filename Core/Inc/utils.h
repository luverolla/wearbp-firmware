/*
 * utils.h
 *
 *  Created on: Jul 12, 2024
 *      Author: Luigi Verolla
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "config.h"

/**
 * @brief Performs in-order add to an array.
 *
 * The array is assumed to be sorted ascending before the operation. If all
 * insertions are performed using this function, the order is automatically
 * maintained.
 *
 * The size is passed as pointer so that it is automatically incremented to
 * account for the new element added
 *
 * @param vec array of data
 * @param n   pointer to the size of the array
 * @param el  the element to add
 */
void vec_ordadd(float* vec, size_t* n, float el);

/**
 * @brief Computes median of an ordered vector
 *
 * The vector is assumed to be already ascending sorted. Use vec_ordadd() to
 * insert elements into the array so that the order is maintained.
 *
 * This operation takes `O(1)` with sorted array. If you decide to do an
 * unordered insertion and to sort the array before calling this function,
 * pay attention to the complexity of your sorting algorithm.
 *
 * @param vec array of data
 * @return the median value of the array
 */
float vec_median(const float* vec, size_t n);



#endif /* INC_UTILS_H_ */
