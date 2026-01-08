#include "Filteringalgorithm.h"

#if FILTER_ALGO_TYPE == FILTER_ALGO_KALMAN

#ifndef FILTER_KALMAN_Q
#define FILTER_KALMAN_Q 0.01f
#endif

#ifndef FILTER_KALMAN_R
#define FILTER_KALMAN_R 1.0f
#endif

static float estimate;
static float error_cov;
static uint8_t kalman_initialized;

void Filteringalgorithm_Init(void)
{
    estimate = 0.0f;
    error_cov = 1.0f;
    kalman_initialized = 0U;
}

long Filteringalgorithm_Process(long sample)
{
    const float measurement = (float)sample;

    if (!kalman_initialized)
    {
        estimate = measurement;
        error_cov = 1.0f;
        kalman_initialized = 1U;
    }
    else
    {
        error_cov += FILTER_KALMAN_Q;
        const float kalman_gain = error_cov / (error_cov + FILTER_KALMAN_R);
        estimate = estimate + kalman_gain * (measurement - estimate);
        error_cov = (1.0f - kalman_gain) * error_cov;
    }

    if (estimate >= 0.0f)
    {
        return (long)(estimate + 0.5f);
    }
    return (long)(estimate - 0.5f);
}

#elif FILTER_ALGO_TYPE == FILTER_ALGO_MOVING_AVG

#include <stdint.h>

#ifndef FILTER_LIMIT_WINDOW
#define FILTER_LIMIT_WINDOW 12U
#endif

#if FILTER_LIMIT_WINDOW == 0
#error "FILTER_LIMIT_WINDOW must be greater than zero"
#endif

static long buffer[FILTER_LIMIT_WINDOW];
static uint8_t buffer_index;
static uint8_t buffer_count;
static int64_t buffer_sum; /* use wider accumulator to reduce overflow risk */

void Filteringalgorithm_Init(void)
{
    buffer_index = 0U;
    buffer_count = 0U;
    buffer_sum = 0;
    for (uint8_t i = 0U; i < FILTER_LIMIT_WINDOW; ++i)
    {
        buffer[i] = 0L;
    }
}

long Filteringalgorithm_Process(long sample)
{
    /* fill buffer until full, then slide window */
    if (buffer_count < FILTER_LIMIT_WINDOW)
    {
        buffer[buffer_index] = sample;
        buffer_sum += sample;
        ++buffer_count;
        buffer_index = (buffer_index + 1U) % FILTER_LIMIT_WINDOW;
        return (long)(buffer_sum / (int64_t)buffer_count);
    }

    buffer_sum -= buffer[buffer_index];
    buffer[buffer_index] = sample;
    buffer_sum += sample;
    buffer_index = (buffer_index + 1U) % FILTER_LIMIT_WINDOW;

    /* average over fixed window; rounding toward zero as in integer division */
    return (long)(buffer_sum / (int64_t)FILTER_LIMIT_WINDOW);
}

#endif
