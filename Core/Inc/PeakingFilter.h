#ifndef INC_PEAKINGFILTER_H_
#define INC_PEAKINGFILTER_H_

#include <math.h>
#include <stdint.h>

typedef struct {
	/* Sample time (s) */
	float sampleTime_s;
	/* Filter inputs (x[0] = current input sample) */
	float x[3];
	/* Filter outputs (y[0] = current filter output sample) */
	float y[3];
	/* x[n] coefficients */
	float a[3];
	/* y[n] coefficients */
	float b[3];

} PeakingFilter;


float PeakingFilter_Update(PeakingFilter *filt, float in);
void PeakingFilter_SetParameters(PeakingFilter *filt, float centerFrequency_Hz, float Q, float gain_dB);
void PeakingFilter_Init(PeakingFilter *filt, float sampleRate_Hz);

#endif /* INC_PEAKINGFILTER_H_ */
