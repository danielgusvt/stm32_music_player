/*
 * PeakingFilter.c
 *
 *  Created on: Jun 17, 2025
 *      Author: Daniel
 */
#include "PeakingFilter.h"

float PeakingFilter_Update(PeakingFilter *filt, float in) {
	/* Shift samples */
	filt->x[2] = filt->x[1];
	filt->x[1] = filt->x[0];
	filt->x[0] = in;

	filt->y[2] = filt->y[1];
	filt->y[1] = filt->y[0];

	/* Compute new filter output */
	filt->y[0] = (filt->a[0] * filt->x[0] + filt->a[1] * filt->x[1] + filt->a[2] * filt->x[2]
			   + 						   (filt->b[1] * filt->y[1] + filt->b[2] * filt->y[2])) * filt->b[0];

	/* Return current output sample */
	return (filt->y[0]);

}

void PeakingFilter_SetParameters(PeakingFilter *filt, float centerFrequency_Hz, float Q, float gain_dB) {
    /* Convert gain from dB to linear scale */
    float boostCut_linear = powf(10.0f, gain_dB / 20.0f);

    /* Convert Hz to rad/s (rps), pre-warp cut-off frequency (bilinear transform) */
    float wcT = 2.0f * tanf(M_PI * centerFrequency_Hz * filt->sampleTime_s);

    /* Intermediate variables for coefficient calculation */
    float wcT_sq = wcT * wcT;
    float Q_wcT = Q * wcT;
    float A_Q_wcT = boostCut_linear * Q_wcT;

    /* Compute filter coefficients */
    filt->a[0] = 4.0f + 2.0f * A_Q_wcT + wcT_sq;
    filt->a[1] = 2.0f * wcT_sq - 8.0f;
    filt->a[2] = 4.0f - 2.0f * A_Q_wcT + wcT_sq;

    /* Compute denominator coefficients (stored as reciprocals and negated) */
    float denom = 4.0f + 2.0f * Q_wcT + wcT_sq;
    filt->b[0] = 1.0f / denom;          /* 1/coefficient */
    filt->b[1] = -(2.0f * wcT_sq - 8.0f); /* -coefficient */
    filt->b[2] = -(4.0f - 2.0f * Q_wcT + wcT_sq); /* -coefficient */
}


void PeakingFilter_Init(PeakingFilter *filt, float sampleRate_Hz) {

	/* Compute sample time */
	filt->sampleTime_s = 1.0f / sampleRate_Hz;

	/* Clear filter memory */
	for (uint8_t n = 0; n < 3; n++) {
		filt->x[n] = 0.0f;
		filt->y[n] = 0.0f;
	}

	/* Calculate 'default' filter coefficients (all-pass) */
	PeakingFilter_SetParameters(filt, 1.0f, 0.5f, 0.0f);
}
