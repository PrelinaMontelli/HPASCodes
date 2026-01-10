#include "Linearization.h"

/*
 * Linearization.c
 * Two selectable non-linear compensation schemes for weight conversion.
 * Switch method via LIN_METHOD_TYPE macro (defaults set in Linearization.h).
 */

/* Piecewise linear table: x in raw counts, y in grams (or desired unit). */
#ifndef LIN_PWL_COUNT
#define LIN_PWL_COUNT 9U
#endif

/* Calibrated points (raw -> g), 原始计数升序：
 * (3361168,87), (3802230,50), (3947725,37), (4149358,20), (4179226,17),
 * (4263327,10), (4297084,7), (4321860,5), (4362460,2), (4388429,0)
 */
static const lin_segment_t lin_table[LIN_PWL_COUNT] = {
    { 3361168.0f, 87.0f, 3802230.0f, 50.0f },
    { 3802230.0f, 50.0f, 3947725.0f, 37.0f },
    { 3947725.0f, 37.0f, 4149358.0f, 20.0f },
    { 4149358.0f, 20.0f, 4179226.0f, 17.0f },
    { 4179226.0f, 17.0f, 4263327.0f, 10.0f },
    { 4263327.0f, 10.0f, 4297084.0f,  7.0f },
    { 4297084.0f,  7.0f, 4321860.0f,  5.0f },
    { 4321860.0f,  5.0f, 4362460.0f,  2.0f },
    { 4362460.0f,  2.0f, 4388429.0f,  0.0f },
};

/* Public API: apply non-linear compensation to raw input. */
float Linearization_Apply(float raw)
{
#if LIN_METHOD_TYPE == LIN_METHOD_POLY
    float y = LIN_POLY_C0 + LIN_POLY_C1 * raw;
#if LIN_POLY_ORDER >= 2
    y += LIN_POLY_C2 * raw * raw;
#endif
#if LIN_POLY_ORDER >= 3
    y += LIN_POLY_C3 * raw * raw * raw;
#endif
    return y;
#elif LIN_METHOD_TYPE == LIN_METHOD_PWL
    /* Clamp outside table range to nearest endpoint. */
    if (raw <= lin_table[0].x0)
    {
        return lin_table[0].y0;
    }
    if (raw >= lin_table[LIN_PWL_COUNT - 1].x1)
    {
        return lin_table[LIN_PWL_COUNT - 1].y1;
    }

    /* Find segment and interpolate. */
    for (uint32_t i = 0; i < LIN_PWL_COUNT; ++i)
    {
        float x0 = lin_table[i].x0;
        float y0 = lin_table[i].y0;
        float x1 = lin_table[i].x1;
        float y1 = lin_table[i].y1;
        if ((raw >= x0) && (raw <= x1))
        {
            float t = (x1 != x0) ? ((raw - x0) / (x1 - x0)) : 0.0f;
            return y0 + t * (y1 - y0);
        }
    }

    /* Fallback (should not reach here): return raw unmodified. */
    return raw;
#endif
}
