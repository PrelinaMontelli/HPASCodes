#include "Linearization.h"

/*
 * Linearization.c
 * Two selectable non-linear compensation schemes for weight conversion.
 * Switch method via LIN_METHOD_TYPE macro (defaults set in Linearization.h).
 */

/* Piecewise linear table: x in raw counts, y in grams (or desired unit). */
#ifndef LIN_PWL_COUNT
#define LIN_PWL_COUNT 4U
#endif

/* Calibrated points (raw -> g): (-4527,80.0), (-1715,30.0), (-597,10.0), (-3,0) */
static const lin_segment_t lin_table[LIN_PWL_COUNT] = {
    { -4527.0f, 80.0f,  -1715.0f, 30.0f },
    { -1715.0f, 30.0f,   -597.0f, 10.0f },
    {  -597.0f, 10.0f,     -3.0f,  0.0f },
    {    -3.0f,  0.0f,      0.0f,  0.0f }, /* clamp near zero */
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
