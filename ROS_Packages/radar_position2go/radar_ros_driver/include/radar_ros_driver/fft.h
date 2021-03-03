#ifndef SRC_FFT_H_
#define SRC_FFT_H_


#include<algorithm>
#include<numeric>
#include<complex>
#include<vector>
#include<map>
#include<math.h>

#define IGNORE_NAN		(555U)

/*
==============================================================================
   3. TYPES
==============================================================================
 */

/**
 * \brief Defines supported FFT input options. Use type FFT_Input_t for this enum.
 * @{
 */
typedef enum
{
   FFT_INPUT_REAL_I    = 0U,  	/**< Only real input I channel for Complex FFT */
   FFT_INPUT_REAL_Q    = 1U,  	/**< Only real input Q channel for Complex FFT */
   FFT_INPUT_COMPLEX   = 2U		/**< Complex input IQ channels for Complex FFT */
}  FFT_Input_t;

/** @} */

/**
 * \brief Defines supported FFT directions. Use type FFT_Direction_t for this enum.
 * @{
 */
typedef enum
{
   FFT_FAST_TIME   = 0U,  	/**< Complex FFT on raw ADC data for Range Spectrum = Fast Time */
   FFT_SLOW_TIME   = 1U  	/**< Complex FFT on fast FFT output signal for Doppler Spectrum = Slow Time */
}  FFT_Direction_t;

typedef struct
{
	double d_phi;

	float target_angle;

}target_angle_data;

class FFT {

public:


};

#endif /* SRC_FFT_H_ */

