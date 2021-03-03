#include <stdio.h>
#include <string.h>
#include <valarray>

// Radar - Host communication
#include "radar_ros_driver/EndpointCalibration.h"
#include "radar_ros_driver/EndpointTargetDetection.h"
#include "radar_ros_driver/EndpointRadarAdcxmc.h"
#include "radar_ros_driver/EndpointRadarBase.h"
#include "radar_ros_driver/EndpointRadarDoppler.h"
#include "radar_ros_driver/EndpointRadarFmcw.h"
#include "radar_ros_driver/EndpointRadarP2G.h"
#include "radar_ros_driver/EndpointRadarIndustrial.h"
#include "radar_ros_driver/Protocol.h"
#include "radar_ros_driver/COMPort.h"

// FFT
#include <iostream>
#include <complex>
#include <fstream>
#include <vector>

// KF
#include <radar_ros_driver/Eigen/Dense>
#include "radar_ros_driver/Hungarian.h"
#include <ros/ros.h>


// Global Definitions
#define FRAME_PERIOD_MSEC 			(150U)    	    // Time period of one frame to capture
#define ANTENNA_SPACING 		    0.0062          // For angle estimation
#define LAMBDA 					    0.0124          // For angle estimation

// Tracking
#define LIFE_TIME_COUNT	            (10U)			// Number of frames after which track is killed
#define GHOST_LIFE_TIME	            (5U)			// Number of frames after which ghost track is killed
#define DELTA_PX_CM		            (80.0f)		    // Size of prediction window, 2 (80),3 (120) bins
#define FRAME_PERIOD_MSEC 		    (150U)

#define RANGE_FFT_SIZE              256
#define DOWN_CHIRP_DURATION         0.0001
#define CHIRP_TO_CHIRP_DELAY        0.0001
#define RANGE_THRESHOLD             700//200
#define DOPPLER_THRESHOLD           50
#define MIN_DISTANCE                0
#define MAX_DISTANCE                4
#define MAX_NUM_TARGETS             5
#define MAX_NUM_TRACKS              5
#define INDEX_ZERO_DOPPLER          17

#define MAX_MEDIAN_FILTER_LEN       17	            // Must be odd
#define NUM_OF_CHIRPS               16
#define CURRENT_NUM_OF_TRACKS       1

#define AUTOMATIC_DATA_FRAME_TRIGGER 0		        // define if automatic trigger is active or not
#define AUTOMATIC_DATA_TRIGER_TIME_US (300)	        // get ADC data each 300us in not automatic trigger mode
#define SPEED_OF_LIGHT              2.998e8

#define CHIRP_DUR_NS                300000
#define	NUM_CHIRPS_PER_FRAME        16
#define DOPPLER_FFT_SIZE            64	            // == NUM_CHIRPS_PER_FRAME!!!
#define	NUM_SAMPLES_PER_CHIRP       64
#define E_SIGNAL_PART               2  			    //ONLY_I = 0 /  ONLY_Q = 1 / I_AND_Q = 2
#define RX_MASK 					3			
// Each available RX antenna is represented by a bit in this mask. 
// If a bit is set, the IF signal received through the according RX antenna is captured during chirp processing.
#define RX_NUM_ANTENNAS             2
#define PEAK_TRESHOLD               0.7

#define MIN_ANGLE_FOR_ASSIGNMENT    50.0
#define IGNORE_NAN		            (555U)
#define ANGLE_QUANTIZATION			(1U)		    // Enable and set the Number of degrees

#define MTI_FILTER_LEN              100
#define MAXIMUM_NUMBER_HISTORY      40

#define LOGNAME_FORMAT 				"data/radar24G_dump_%Y%m%d_%H%M%S.dump"
#define LOGNAME_SIZE 				100

#define PI							3.14159265358979323846

/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
 * \brief Data structure for Median filtering.
 * @{
 */
typedef struct
{
	uint8_t  is_full;
	uint32_t median_filter_len;
	float    buffer[MAX_MEDIAN_FILTER_LEN];
} Median_Filtering_t;

/**
 * \brief Data structure for current measurements used in data association.
 * @{
 */
typedef struct
{
	uint16_t is_associated;
	float    strength;
	float    range;
	float    speed;
	float	 angle;
	float    rx1_angle_arg_re;
	float    rx1_angle_arg_im;
	float    rx2_angle_arg_re;
	float    rx2_angle_arg_im;
} Measurement_elem_t;

/**
 * \brief Data structure for Track parameters used in tracking.
 * @{
 */
typedef struct
{
	uint8_t  track_id;
	uint8_t  is_alived;
	uint16_t speed_count;
	uint16_t range_change_flag;
	uint16_t lifetime_counter;
	uint32_t measurement_counter;
	float    strength;
	float    range;
	float    speed;
	float    speed_th;
	float    angle;
	float    rx1_angle_arg_re[NUM_OF_CHIRPS];
	float    rx1_angle_arg_im[NUM_OF_CHIRPS];
	float    rx2_angle_arg_re[NUM_OF_CHIRPS];
	float    rx2_angle_arg_im[NUM_OF_CHIRPS];
	float    d_phi;
} Tracking_Params_t;

/**
 * \brief Data structure for Tracking List.
 * @{
 */
typedef struct
{
	uint32_t num_of_tracks;
	uint32_t max_num_of_tracks;
	Tracking_Params_t elems[CURRENT_NUM_OF_TRACKS];
} tracking_list_t;

/*
 * Algorithm settings structure
 */
typedef struct
{
	uint8_t    isUpdated;
	uint8_t    isChecked;

	uint32_t   max_number_of_targets;
	uint32_t   max_number_of_tracks;

	uint32_t   num_of_tracks;
	uint32_t   mvg_avg_len;
	uint32_t   median_filter_len;
	uint32_t   mti_filter_len;
	uint32_t   mti_filter_enable;
	uint32_t   range_offset_cm;    // provided via calibration endpoint/struct!
	int16_t    angle_offset_deg;   // provided via calibration endpoint/struct!

	uint32_t   min_distance_cm;
	uint32_t   max_distance_cm;
	uint32_t   range_detection_threshold;

	uint32_t   min_speed_kmh;
	uint32_t   max_speed_kmh;
	float      wave_length_ant_spacing_ratio;
	float	   min_angle_for_track_assignment;
} algo_settings_t;


typedef struct
{
	double d_phi;
	float target_angle;
}target_angle_data;

typedef struct
{
	double x;
	double y;
}target_history;


typedef struct
{
	int index;
	double peak_val;
}target_peak;


using namespace std;



class ofxRadar24Ghz {

	public:
		void setup();
		// void draw();
		void update();

		// ofxVectorGraphics output;

		//TRACKING
		Median_Filtering_t *median_angle_arr;//[CURRENT_NUM_OF_TRACKS];
		//Median_Filtering_t median_angle_arr[CURRENT_NUM_OF_TRACKS];;//[CURRENT_NUM_OF_TRACKS];
		float *rx_angle_fft;//[2*DOPPLER_FFT_SIZE];
		float *rx_angle_fft_spectrum;//[DOPPLER_FFT_SIZE];
		uint32_t frame_period_usec;
		algo_settings_t *cp_algo_settings;
		tracking_list_t *tracking_list;
		target_history * pos_history;
		int n_points_history;

		// frame initialize memory
		int num_chirps;
		int num_samples_per_chirp;
		int esignalpart;
		int rx_mask;
		int num_antennas;

		int radar_handle = 0;
		int num_of_ports = 0;
		char comp_port_list[256];
		char* comport;
		const char *delim = ";";

		int res;
		int protocolHandle;
		int endpointRadarBase;
		bool acq_started;


		// Algorithm
		bool enable_tracking;
		double speed_of_light;
		double fC;
		double PRT;
		int fs;
		int BW;
		int range_fft_size;
		int doppler_fft_size;
		int range_threshold;
		int doppler_threshold;
		int min_distance;
		int max_distance;
		int max_num_targets;
		double lambda;
		double hz_to_mps_constant;
		double if_scale;
		double *range_window_func;
		double *dopper_window_func;
		int r_max;
		double dist_per_bin;
		vector<double> array_bin_range;
		double fD_max;
		double fD_per_bin;
		vector<double> array_bin_fD;

		bool enable_mti_filtering;
		complex<double> * range_fft_spectrum_hist1; // FOR MTI
		complex<double> * range_fft_spectrum_hist2; // FOR MTI
		complex<double> * fft_1; // range FFT
		complex<double> * fft_2; // range FFT


		// FFT
		//fftw_complex *out;
		//fftw_plan plan_forward;
		double *adc_real_tx1rx1;	// REAL
		double *adc_imag_tx1rx1;	// IMG
		double *adc_real_tx1rx2;	// REAL
		double *adc_imag_tx1rx2;	// IMG

		// ADC I AND Q two antennas
		float *full_data_block;
		float *temperature;
		void  *frame_format_current;
		void  *device_info;
		void  *fmcw_cfg;
		uint32_t  *chirp_duration_ns;
		uint32_t  *min_frame_interval_us;
		uint32_t  *tx_power_001dBm;


		// signal magnitude and phase shift
		double *matrix_mag_1;
		double *matrix_phase_1;
		double *matrix_mag_2;
		double *matrix_phase_2;

		// ======== Kalman filter matrices =========
		void kalmanfilter(Eigen::VectorXd& x_hat, Eigen::VectorXd& y, Eigen::MatrixXd& P);
		Eigen::MatrixXd A, H, R, P0, I, G;
		//Eigen::VectorXd Q;
		Eigen::MatrixXd* P = new Eigen::MatrixXd[5];
		Eigen::VectorXd* x_hat = new Eigen::VectorXd[5];

		// ======== Data association custom: =======
		Tracking_Params_t *tracking_list2;
		void data_association2(Tracking_Params_t *track_lst, vector<Measurement_elem_t>& measurements, uint16_t num_of_targets,
			algo_settings_t *cp_algo_settings, double duration_sec);
		uint8_t get_next_free_trackID2(Tracking_Params_t *p_tracks );
		uint32_t init_track(Tracking_Params_t *track_lst, Measurement_elem_t& measurement, uint8_t id, uint16_t angle_offset_deg,
				float wave_length_ant_spacing_ratio, float min_angle);
		void solve_Hungarian(Eigen::MatrixXd& Cost, vector<int>& assignment);
		void update_track_measure(Tracking_Params_t *track_lst, Measurement_elem_t& measurement, uint8_t id, uint16_t angle_offset_deg,
				float wave_length_ant_spacing_ratio);

		// ======== Velocity obstacles =============
		bool true_VO = true;
		void velocity_obstacles(Tracking_Params_t *track_lst);
		int8_t avoid_state;
		// Body velocity state (from subscriber to msp):
		double v_xa = 0;
		double v_ya = 0;
		// Desired velocity state (to publisher msp):
		double v_xa_des = 0;
		double v_ya_des = 0;

		// Gating distance:
		double d_abs = 0.4;//meters
		// Frame number
		uint32_t frame_id = 0;
		// Measurement vector to be buffered and fed into tracking algo
		vector<Measurement_elem_t> measurements;
		// true angle offset
		double angle_correction = -8.0;
		// Covariance initiation factor:
		double cov_factor = 1;
		double time_stamp = 0;
		double time_rec = 0;
		double delay_s;
		// =========================================

		bool plotRaw;

		// FFT maps
		vector<vector<complex<double>>> range_tx1rx1;
		vector<vector<complex<double>>> range_tx1rx2;

		double * distance_m;

		// DOPPLER
		vector<vector<complex<double>>> range_doppler_tx1rx1;
		vector<vector<complex<double>>> range_doppler_tx1rx2;
		vector<vector<complex<double>>> rangeFFT1;
		vector<vector<complex<double>>> rangeFFT2;


		// END
		vector<Measurement_elem_t> current_targets; // here the content

		int countSetBits(unsigned int n);

		// void drawMatrix(int posx, int posy, double * matrix, string title, int gain);
		// void drawRangeFFT(int posx, int posy, complex<double> * range_fft, string title, target_peak* peaks);

		// void drawTargets();
		void changeTracking();
		void changeMTI();
		void changeRecording();
		void changeLoadData();
		void clearTargets();

		void f_search_peak(double * fft_spectrum, int search_lenght, double threshold, int max_target_count,
				double min_distance,  double max_distance, double dist_per_bin, target_peak *tgt_range);
		void startRadarUSB();
		//vector<vector<double>> found_peaks_1;
		//vector<vector<double>> found_peaks_2;

		target_peak *tgt_range1;
		target_peak *tgt_range2;


		double calculateBeatFreq(double distance_m, double bandwidth_hz, double speed_of_light, double ramp_time_s);
		void getcolor(int p, int norm_p, float * r, float * g, float * b);

		vector<bool> recordingstatus;
		ofstream binaryDataOut;
		ifstream bindayDataIn;
		bool isrecording;

		bool isloaddata;
		bool file_loaded;
		bool repeat_mode;

		bool islive;


		int 	radar_auto_connect();  // just pick the only radar available
		void 	print_status_code( int32_t protocol_handle, int32_t status);

		double blackman(double i, double N) {
			double a0 = 0.42;
			double a1 = 0.5;
			double a2 = 0.08;
			double f = 6.283185307179586*i/(N-1);
			return a0 - a1 * cos(f) + a2*cos(2.0*f);
		}


		/***************************************************************************
		 calculate a chebyshev window of size N, store coeffs in out as in out
		-out should be array of size N
		-atten is the required sidelobe attenuation (e.g. if you want -60dB atten, use '60')
		***************************************************************************/
		void cheby_win(double *out, int N, float atten){
			int nn, i;
			double M, n, sum = 0, max=0;
			double tg = pow(10,atten/20);  /* 1/r term [2], 10^gamma [2] */
			double x0 = cosh((1.0/(N-1))*acosh(tg));
			M = (N-1)/2;
			if(N%2==0) M = M + 0.5; /* handle even length windows */
			for(nn=0; nn<(N/2+1); nn++){
				n = nn-M;
				sum = 0;
				for(i=1; i<=M; i++){
					sum += cheby_poly(N-1,x0*cos(PI*i/N))*cos(2.0*n*PI*i/N);
				}
				out[nn] = tg + 2*sum;
				out[N-nn-1] = out[nn];
				if(out[nn]>max)max=out[nn];
			}
			for(nn=0; nn<N; nn++) out[nn] /= max; /* normalise everything */
			return;
		}

		/**************************************************************************
		This function computes the chebyshev polyomial T_n(x)
		***************************************************************************/
		double cheby_poly(int n, double x){
			double res;
			if (fabs(x) <= 1) res = cos(n*acos(x));
			else              res = cosh(n*acosh(x));
			return res;
		}

		////////////////////////////////////
		// TRACKING ALGORITHM
		////////////////////////////////////

		/// DATASTORE
		/*
		void data_association(tracking_list_t *p_tracks, Measurement_elem_t* target_measurements, uint16_t num_of_targets,
				algo_settings_t *cp_algo_settings, uint32_t frame_period_usec, uint32_t num_of_chirps );

		static uint8_t get_next_free_trackID(tracking_list_t *p_tracks);

		//===========================================================================
		*/
		void clear_track_elem(Tracking_Params_t* track_ptr){
			memset(track_ptr, 0, sizeof(Tracking_Params_t));
			track_ptr->d_phi = IGNORE_NAN;
		}
		/*

		uint32_t assign_track(tracking_list_t *p_tracks,
				Measurement_elem_t* target_measurement, uint8_t id, uint16_t angle_offset_deg,
				float wave_length_ant_spacing_ratio, float min_angle);

		void update_track(Tracking_Params_t* track_ptr,
				Measurement_elem_t* target_measurement, uint8_t track_id,
				float px_track_predict,
				float range_detection_threshold, int16_t angle_offset_deg, uint32_t num_of_chirps,
				float wave_length_ant_spacing_ratio);
		*/

		static int compare_float(const void* a, const void* b);

		static float median_filtering(Median_Filtering_t *track_median_arr, float new_input);

		double compute_angle(complex<double> z1, complex<double> z2, float wave_length_ant_spacing_ratio){
			double angle_rx1 = atan2(z1.imag(), z1.real()); // phase of received signal for rx1
			double angle_rx2 = atan2(z2.imag(), z2.real()); // phase of received signal for rx2

			double d_phi = angle_rx1 - angle_rx2;
			if(d_phi <= 0){
				d_phi = d_phi + 2*PI;
			}
			d_phi = d_phi - PI;
			double target_angle = asin(d_phi * wave_length_ant_spacing_ratio  / (2*PI));

			return (target_angle*(180/PI)); // deg

		}

		double get_phase(float real, float imag);

		/**
		 * \brief  This function computes the FFt signal out of raw ADC samples.
		 *
		 *  Internally it computes mean of respective I & Q signal and subtract it before applying IF scaling and Windowing.
		 *  Afterwards computes the FFT signal and returns the Nf number of complex samples.
		 *
		 * \param[in]	*i_data		Pointer of type signed 16-bit integer, containing the address of the I data buffer
		 * \param[in]	*q_data		Pointer of type signed 16-bit integer, containing the address of the Q data buffer
		 * \param[in]	Nd			Unsigned 16-bit integer, containing the size of raw ADC IQ data buffer
		 * \param[in]	Nf			Unsigned 16-bit integer, containing the size of FFT complex values array
		 * \param[in]	if_scale	Floating point scale applied to the FFT spectrum to enhance the visibility of targets
		 * \param[in]	fft_type	Complex or Real input FFT to be computed defined by \ref FFT_Input_t
		 * \param[in]	fft_direction	Fast or Slow FFT to be computed defined by \ref FFT_Direction_t
		 *
		 * \param[out]  *i_mean		Pointer to a floating point value, containing the mean of the I channel
		 * \param[out]  *q_mean		Pointer to a floating point value, containing the mean of the Q channel
		 * \param[out]  *complex_fft_signal		Pointer to a floating point array, to return the complex FFT signal in interleaved I&Q format.
		 *
		 */
		//void compute_fft_signal(float* i_data, float* q_data, uint16_t Nd, uint16_t Nf, float if_scale,
		//						FFT_Input_t fft_type, FFT_Direction_t fft_direction,
		//						float* i_mean, float* q_mean, float* complex_fft_signal);

		/**
		 * \brief  This function computes the FFt spectrum out of raw ADC samples.
		 *
		 *  Internally it computes mean of respective I & Q signal and subtract it before applying IF scaling and Windowing.
		 *  Afterwards computes the FFT signal and returns the Nf number of real samples as FFT spectrum.
		 *
		 * \param[in]	*fft_input_signal		Pointer of type float, containing the address of the Complex FFT signal with interleaved IQ
		 * \param[in]	Nf						Unsigned 32-bit integer, containing the size of FFT complex values array
		 *
		 * \param[out]  *fft_output_spectrum	Pointer to a floating point array, to return the real valued FFT spectrum.
		 *
		 */
		//void compute_fft_spectrum(float* fft_input_signal, uint32_t Nf, float * fft_output_spectrum);

		/**
		 * \brief  This function computes the angle (in degrees) using two receive antennas IQ signals.
		 *
		 * The distance between the two RX-antennas (7.5mm) results in a maximum measurable angle of +-30°
		 * Can be calculated as:  +-phi = arcsin((+-Pi*lambda)/(2*Pi*distanceRX))
		 *
		 * \param[in]	if1_i				Real component of FFT signal for the desired detected target bin for receive antenna 1.
		 * \param[in]	if1_q				Imaginary component of FFT signal for the desired detected target bin for receive antenna 1.
		 * \param[in]	if2_i				Real component of FFT signal for the desired detected target bin for receive antenna 2.
		 * \param[in]	if2_q				Imaginary component of FFT signal for the desired detected target bin for receive antenna 2.
		 * \param[in]	angle_offset_deg	correction offset of angle, from calibration data.
		 * \param[in]	wave_length_ant_spacing_ratio	ratio bewetween wave_length and rx antenna spacing
		 *
		 * \return	Angle of respective detected target in units of degrees with quadrant adjusted.
		 *
		 */
		target_angle_data compute_angle_d(float if1_i, float if1_q, float if2_i, float if2_q, double d_old, int16_t angle_offset_deg, float wave_length_ant_spacing_ratio);



		////////////////////////////////////
		// callback functions COMMUNICATION
		////////////////////////////////////

		// query frame format
		void get_frame_format(int32_t protocol_handle,
				uint8_t endpoint,
				Frame_Format_t* frame_format);


		static void 	received_frame_data(void* context,
							int32_t protocol_handle,
							uint8_t endpoint,
							const Frame_Info_t* frame_info);

		static void received_frame_format(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				const Frame_Format_t* frame_format);

		static void received_temperature(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				uint8_t temp_sensor,
				int32_t temperature_001C);

		static void get_chirp_duration(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				uint32_t chirp_duration_ns);

		static void get_device_info(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				const Device_Info_t* device_info);

		static void get_tx_power(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				uint8_t tx_antenna,
				int32_t tx_power_001dBm);


		static void get_bw_sec(void* context,
			int32_t protocol_handle,
			uint8_t endpoint,
			uint32_t bandwidth_per_second);


		static void set_fmcw_conf(void* context,
				int32_t protocol_handle,
				uint8_t endpoint,
				const Fmcw_Configuration_t*
				fmcw_configuration);

		static void get_min_frame_interval(void* context,
										int32_t protocol_handle,
										uint8_t endpoint,
										uint32_t min_frame_interval_us);


		// INLINE DEFINED DSP
		int log2fft(int N)    //funzione per calcolare il logaritmo in base 2 di un intero
		{
		int k = N, i = 0;
		while(k) {
			k >>= 1;
			i++;
		}
		return i - 1;
		}

		int checkfft(int n)    //usato per controllare se il numero di componenti del vettore di input è una potenza di 2
		{
		return n > 0 && (n & (n - 1)) == 0;
		}

		int reversefft(int N, int n)    //calcola il reverse number di ogni intero n rispetto al numero massimo N
		{
		int j, p = 0;
		for(j = 1; j <= log2fft(N); j++) {
			if(n & (1 << (log2fft(N) - j)))
			p |= 1 << (j - 1);
		}
		return p;
		}

		void ordinafft(complex<double>* f1, int N)     //dispone gli elementi del vettore ordinandoli per reverse order
		{
		complex<double> f2[N];
		for(int i = 0; i < N; i++)
			f2[i] = f1[reversefft(N, i)];
		for(int j = 0; j < N; j++)
			f1[j] = f2[j];
		}

		void transformfft(complex<double>* f, int N)     //calcola il vettore trasformato
		{
			ordinafft(f, N);    //dapprima lo ordina col reverse order
			complex<double> W[N / 2]; //vettore degli zeri dell'unità.
										//Prima N/2-1 ma genera errore con ciclo for successivo
									//in quanto prova a copiare in una zona non allocata "W[N/2-1]"
			W[1] = polar(1., -2. * M_PI / N);
			W[0] = 1;
			for(int i = 2; i < N / 2; i++)
				W[i] = pow(W[1], i);
			int n = 1;
			int a = N / 2;
			for(int j = 0; j < log2fft(N); j++) {
				for(int i = 0; i < N; i++) {
				if(!(i & n)) {
					/*ad ogni step di raddoppiamento di n, vengono utilizzati gli indici */
					/*'i' presi alternativamente a gruppetti di n, una volta si e una no.*/
					complex<double> temp = f[i];
					complex<double> Temp = W[(i * a) % (n * a)] * f[i + n];
					f[i] = temp + Temp;
					f[i + n] = temp - Temp;
				}
				}
				n *= 2;
				a = a / 2;
			}
		}

		void FFT(complex<double>* f, int N, double d)
		{
		transformfft(f, N);
		for(int i = 0; i < N; i++)
			f[i] *= d; //moltiplica il vettore per il passo in modo da avere il vettore trasformato effettivo
		}


	};
