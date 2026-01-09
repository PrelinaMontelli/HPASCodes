#ifndef LINEARIZATION_H
#define LINEARIZATION_H

#include <stdint.h>

/* Method selection */
#define LIN_METHOD_POLY 1
#define LIN_METHOD_PWL  2

#ifndef LIN_METHOD_TYPE
#define LIN_METHOD_TYPE LIN_METHOD_PWL
#endif

#if (LIN_METHOD_TYPE != LIN_METHOD_POLY) && (LIN_METHOD_TYPE != LIN_METHOD_PWL)
#error "Unsupported LIN_METHOD_TYPE"
#endif

/* Polynomial coefficients: default to current linear calibration */
#ifndef LIN_POLY_ORDER
#define LIN_POLY_ORDER 1U
#endif
#if LIN_POLY_ORDER > 3
#error "LIN_POLY_ORDER > 3 not supported"
#endif
/* Recalibrated from latest data: (raw, g) ≈ (-5,0), (-573,9.55), (-1686,29.21), (-4505,78.85), (-4600,80.42) */
#ifndef LIN_POLY_C0
#define LIN_POLY_C0 (-0.344f)
#endif
#ifndef LIN_POLY_C1
#define LIN_POLY_C1 (-0.01755f)
#endif
#ifndef LIN_POLY_C2
#define LIN_POLY_C2 0.0f
#endif
#ifndef LIN_POLY_C3
#define LIN_POLY_C3 0.0f
#endif

/* Piecewise linear default table points (can be overridden in .c) */
typedef struct
{
    float x0;
    float y0;
    float x1;
    float y1;
} lin_segment_t;

float Linearization_Apply(float raw);

#endif /* LINEARIZATION_H */
