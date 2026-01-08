#ifndef FILTERINGALGORITHM_H
#define FILTERINGALGORITHM_H

#include <stdint.h>

#define FILTER_ALGO_KALMAN 1
#define FILTER_ALGO_MOVING_AVG 2

#ifndef FILTER_ALGO_TYPE
#define FILTER_ALGO_TYPE FILTER_ALGO_MOVING_AVG
#endif

#if (FILTER_ALGO_TYPE != FILTER_ALGO_KALMAN) && (FILTER_ALGO_TYPE != FILTER_ALGO_MOVING_AVG)
#error "Unsupported FILTER_ALGO_TYPE"
#endif

void Filteringalgorithm_Init(void);
long Filteringalgorithm_Process(long sample);

#endif
