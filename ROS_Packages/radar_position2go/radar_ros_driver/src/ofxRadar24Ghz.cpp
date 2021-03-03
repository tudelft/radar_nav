#include "radar_ros_driver/ofxRadar24Ghz.h"

//--------------------------------------------------------------
void ofxRadar24Ghz::setup() {

	// via definitions
	num_chirps = NUM_CHIRPS_PER_FRAME;
	num_samples_per_chirp = NUM_SAMPLES_PER_CHIRP;
	esignalpart = E_SIGNAL_PART;
	rx_mask = RX_MASK;
	num_antennas = countSetBits(rx_mask);
	speed_of_light = SPEED_OF_LIGHT;// SPEED OF LIGHT

	// allocate memory for callbacks
	full_data_block = (float *)malloc(num_antennas * num_chirps * num_samples_per_chirp * 2 * sizeof(float) );	// I and Q
	temperature = (float *)malloc(1 * sizeof(float));
	frame_format_current = (Frame_Format_t *)malloc(1 * sizeof(Frame_Format_t));
	device_info = (Device_Info_t *)malloc(1 * sizeof(Device_Info_t));
	chirp_duration_ns = (uint32_t*)malloc(1*sizeof(uint32_t));
	min_frame_interval_us = (uint32_t*)malloc(1*sizeof(uint32_t));
	tx_power_001dBm = (uint32_t*)malloc(1*sizeof(uint32_t));
	distance_m = (double *)malloc(num_samples_per_chirp * sizeof(double));
	fmcw_cfg = (Fmcw_Configuration_t*)malloc(1 * sizeof(Fmcw_Configuration_t));
	tgt_range1 = (target_peak*)malloc(MAX_NUM_TARGETS * sizeof(target_peak));
	tgt_range2 = (target_peak*)malloc(MAX_NUM_TARGETS * sizeof(target_peak));
	// init index to be -1
	for(size_t tp=0;tp<MAX_NUM_TARGETS; tp++){
		tgt_range1[tp].index = -1;
		tgt_range1[tp].peak_val = 0.0;
		tgt_range2[tp].index = -1;
		tgt_range2[tp].peak_val = 0.0;
	}
	range_fft_spectrum_hist1 = (complex<double>*)malloc(RANGE_FFT_SIZE * sizeof(complex<double>) );
	range_fft_spectrum_hist2 = (complex<double>*)malloc(RANGE_FFT_SIZE * sizeof(complex<double>) );
	fft_1 = (complex<double>*)malloc(RANGE_FFT_SIZE * sizeof(complex<double>) );
	fft_2 =  (complex<double>*)malloc(RANGE_FFT_SIZE * sizeof(complex<double>) );
	// init to zero
	for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < RANGE_FFT_SIZE; SAMPLE_NUMBER++){
		range_fft_spectrum_hist1[SAMPLE_NUMBER] = (0.0);
		range_fft_spectrum_hist2[SAMPLE_NUMBER] = (0.0);
		fft_1[SAMPLE_NUMBER] = (0.0);
		fft_2[SAMPLE_NUMBER] = (0.0);
	}
	// data
	adc_real_tx1rx1 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));
	adc_imag_tx1rx1 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));
	adc_real_tx1rx2 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));
	adc_imag_tx1rx2 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));
	//plotting
	matrix_mag_1 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));
	matrix_mag_2 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));
	matrix_phase_1 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));
	matrix_phase_2 = (double *)malloc(NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP* sizeof(double));

	// generals
	radar_handle = 0;
	num_of_ports = 0;
	res = -1;
	protocolHandle = 0;
	endpointRadarBase = 0;

	// START CONNECTION TO RADAR VIA USB
	startRadarUSB();

	Device_Info_t *this_device_infos = (Device_Info_t *) (device_info);
 	fC = ((double)this_device_infos->max_rf_frequency_kHz + (double)this_device_infos->min_rf_frequency_kHz)/2.0 * 1e3;

	fs = 426666; // Adcxmc configuration
	PRT = 0.0005;//chirp_duration_ns + DOWN_CHIRP_DURATION + CHIRP_TO_CHIRP_DELAY;
	BW = 200000000; // in HZ
	range_fft_size = RANGE_FFT_SIZE;
	doppler_fft_size = DOPPLER_FFT_SIZE;
	range_threshold = RANGE_THRESHOLD;
	doppler_threshold = DOPPLER_THRESHOLD;
	min_distance = MIN_DISTANCE; // m
	max_distance = MAX_DISTANCE;
	max_num_targets = MAX_NUM_TARGETS;
	lambda = SPEED_OF_LIGHT/fC;

	hz_to_mps_constant=lambda/2.0;
	if_scale= 16 * 3.3*range_fft_size/num_samples_per_chirp;
	// REANGE WINDOWING
	range_window_func = (double*)malloc(NUM_SAMPLES_PER_CHIRP*sizeof(double));
	for(double ij=0;ij<num_samples_per_chirp;ij++){
		double val = 2*blackman(ij, num_samples_per_chirp);
		range_window_func[(int)ij] = (val);
		//printf("val %f", val);
	}

	// DOPPLER
	dopper_window_func = (double*)malloc(DOPPLER_FFT_SIZE*sizeof(double));
	cheby_win(dopper_window_func, DOPPLER_FFT_SIZE, 120);

	r_max = NUM_SAMPLES_PER_CHIRP*SPEED_OF_LIGHT/ (2*BW); // max range!
	dist_per_bin = (double)r_max / (double)range_fft_size;
	double val_j;
	for(int jj=0.0; jj< range_fft_size; jj++){
		val_j = jj*dist_per_bin;
		array_bin_range.push_back(val_j);
	}

	fD_max = (double)((1.0) / (2.0*(double)PRT));
	fD_per_bin = (double)fD_max /(double)(doppler_fft_size/2);
	for(int jj=0;jj< doppler_fft_size ;jj++){
		double val = (double)((double)jj - ((double)doppler_fft_size /2.0) - 1)*-1.0*(double)fD_per_bin*(double)hz_to_mps_constant;
		array_bin_fD.push_back(val);
	}

	// doppler FFT MATRIX, just one frame
	for(int h = 0; h < RANGE_FFT_SIZE; h++ ){
		vector<complex<double>> this_ll;
		for(unsigned int i=0; i< DOPPLER_FFT_SIZE; i++){
			complex<double> cv;
			cv =(0.0);
			//cv.imag(0.0);
			this_ll.push_back(cv);
		}
		range_doppler_tx1rx1.push_back(this_ll);
		range_doppler_tx1rx2.push_back(this_ll);
		rangeFFT1.push_back(this_ll);
		rangeFFT2.push_back(this_ll);
	}

	// ================= Kalman Filter ==================
	//Eigen::MatrixXd H(2, 4);
	Eigen::MatrixXd H(3, 4);
	//Eigen::MatrixXd R(2, 2);
	Eigen::MatrixXd R(3, 3);
	Eigen::MatrixXd I(4, 4);
	//Eigen::VectorXd Q(2);
	Eigen::VectorXd x_template(4);
	for (int i=0;i<5;i++){
		P[i] = P0;
		x_hat[i] = x_template;
	}
	H << 1,0,0,0 , 0,1,0,0 , 0,0,1,0;
	R << 1,0,0 , 0,1,0 , 0,0,1;// sigma_m ^ 2
	I << 1,0,0,0 , 0,1,0,0 , 0,0,1,0 , 0,0,0,1;
	this->H = H;
	this->R = R;
	this->I = I;
	//printf("\n-------------\n%f\n---------------", A[1]);

	// ================= TRACKING =====================
	enable_tracking = true; // press
	//true_VO = true;
	enable_mti_filtering = false;
	median_angle_arr = (Median_Filtering_t*)malloc(CURRENT_NUM_OF_TRACKS*sizeof(Median_Filtering_t));//[CURRENT_NUM_OF_TRACKS];
	for(size_t this_median=0; this_median<CURRENT_NUM_OF_TRACKS; this_median++){
		median_angle_arr[this_median].is_full = 0;
	}

	rx_angle_fft = (float*)malloc(2*DOPPLER_FFT_SIZE*sizeof(float));//[2*DOPPLER_FFT_SIZE];
	rx_angle_fft_spectrum = (float*)malloc(1*DOPPLER_FFT_SIZE*sizeof(float));//[DOPPLER_FFT_SIZE];
	frame_period_usec = FRAME_PERIOD_MSEC;

	// TRACKING ALGORITHM SETTINGS
	cp_algo_settings = (algo_settings_t*)malloc(1*sizeof(algo_settings_t));
	// settings
	cp_algo_settings->max_number_of_targets = MAX_NUM_TARGETS;
	cp_algo_settings->max_number_of_tracks = MAX_NUM_TARGETS;
	cp_algo_settings->num_of_tracks = 0;
	cp_algo_settings->mvg_avg_len = 2;
	cp_algo_settings->median_filter_len = MAX_MEDIAN_FILTER_LEN;
	cp_algo_settings->mti_filter_len = MTI_FILTER_LEN; // MTI_FILTER_LEN
	cp_algo_settings->mti_filter_enable = 0;
	cp_algo_settings->range_offset_cm = 0;    // provided via calibration endpoint/struct!
	cp_algo_settings->angle_offset_deg = 0;   // provided via calibration endpoint/struct!
	cp_algo_settings->min_distance_cm = MIN_DISTANCE*100;
	cp_algo_settings->max_distance_cm = MAX_DISTANCE*100;
	cp_algo_settings->range_detection_threshold = RANGE_THRESHOLD;
	cp_algo_settings->min_speed_kmh = 0.0;
	cp_algo_settings->max_speed_kmh = 4.0;
	cp_algo_settings->wave_length_ant_spacing_ratio= LAMBDA/ANTENNA_SPACING;
	cp_algo_settings->min_angle_for_track_assignment=MIN_ANGLE_FOR_ASSIGNMENT;
	// init memory for tracking results
	tracking_list = (tracking_list_t*)malloc(MAX_NUM_TARGETS*sizeof(tracking_list_t));
	for(size_t i=0; i<MAX_NUM_TARGETS;i++){
		clear_track_elem(tracking_list[i].elems);
		tracking_list[i].num_of_tracks = 0;
		tracking_list[i].max_num_of_tracks = 1; //
	}
	tracking_list2 = (Tracking_Params_t*)malloc(MAX_NUM_TARGETS*sizeof(Tracking_Params_t));
	for(size_t i=0; i<MAX_NUM_TARGETS;i++){
		clear_track_elem(&tracking_list2[i]);
		//memset(&tracking_list2[i], 0, sizeof(Tracking_Params_t));
	}
	pos_history = (target_history*)malloc(MAXIMUM_NUMBER_HISTORY*sizeof(target_history));
	n_points_history = 0;// start counting from zero to MAXIMUM_NUMBER_HISTORY;

	// recording files
	isrecording = false;
	// loading from file
	isloaddata = false;
	file_loaded = false;
	repeat_mode = false;
	acq_started = false;

}

double ofxRadar24Ghz::calculateBeatFreq(double distance_in_m, double bandwidth_hz, double speed_of_light, double ramp_time_s){
	return(2.0*distance_in_m*bandwidth_hz)/(speed_of_light*ramp_time_s);
}

void ofxRadar24Ghz::changeTracking(){
	enable_tracking = !enable_tracking;
}

void ofxRadar24Ghz::changeMTI(){
	enable_mti_filtering = !enable_mti_filtering;
}

// void ofxRadar24Ghz::changeRecording(){

// 	if(isrecording ){
// 		binaryDataOut.close();
// 		ofLog(OF_LOG_NOTICE, "STOP recording data");
// 	}else{
//     	// open new file if string is not open
//     	if(!binaryDataOut.is_open()){
//     		static char name[LOGNAME_SIZE];
//     		time_t now = time(0);
//     		strftime(name, sizeof(name), LOGNAME_FORMAT, localtime(&now));
//     		binaryDataOut.open(name, std::ios_base::app);
//     		ofLog(OF_LOG_NOTICE, "START recording data");
//     	}
// 	}
// 	isrecording = !isrecording;

// }

void ofxRadar24Ghz::startRadarUSB(){

	// open COM port
	protocolHandle = radar_auto_connect();

	// get endpoint ids
	if (protocolHandle >= 0)
	{
		for (int i = 1; i <= protocol_get_num_endpoints(protocolHandle); ++i) {
			// current endpoint is radar base endpoint
			if (ep_radar_base_is_compatible_endpoint(protocolHandle, i) == 0) {
				endpointRadarBase = i;
				continue;
			}
		}
	}

	if (endpointRadarBase >= 0)
	{
		// compatible in all means
		uint32_t is_compatible = ep_radar_base_is_compatible_endpoint(protocolHandle,endpointRadarBase);
		// ------------------------------ --------------------------------------------------
		// ofLog(OF_LOG_WARNING, "EP RADAR IS COMPATIBLE"); 
		// ------------------------------ --------------------------------------------------
		print_status_code( protocolHandle, is_compatible);

		// callback get device info
		ep_radar_base_set_callback_device_info(this->get_device_info, device_info);
		// callback for frame format messages.
		ep_radar_base_set_callback_frame_format(this->received_frame_format, frame_format_current);
		// callback min frame interval
		ep_radar_base_set_callback_min_frame_interval(this->get_min_frame_interval, min_frame_interval_us);
		// callback chirp_duration
		ep_radar_base_set_callback_chirp_duration(this->get_chirp_duration, chirp_duration_ns);
		// register call back functions for adc data
		ep_radar_base_set_callback_data_frame(this->received_frame_data, full_data_block);
		// register call back for tx power read
		ep_radar_base_set_callback_tx_power(this->get_tx_power, tx_power_001dBm);

		// get device info
		uint32_t dev_info_status = ep_radar_base_get_device_info(protocolHandle,endpointRadarBase);
		print_status_code( protocolHandle, dev_info_status);
		// get power
		int32_t answer = ep_radar_base_get_tx_power(protocolHandle, endpointRadarBase, 0);
		print_status_code(protocolHandle, answer);
		// get current frame format
		Frame_Format_t* frame_format_now;
		frame_format_now = (Frame_Format_t *)malloc(1 * sizeof(Frame_Format_t));
		this->get_frame_format(protocolHandle, endpointRadarBase, frame_format_now);

		/* If the frame format contains a 0, this makes no sense. */
		if ((frame_format_now->rx_mask == 0) ||
		  (frame_format_now->num_chirps_per_frame  == 0) ||
			(frame_format_now->num_samples_per_chirp == 0) ||
			  (frame_format_now->num_chirps_per_frame  > (uint32_t)num_chirps) ||
				(frame_format_now->num_samples_per_chirp > (uint32_t)num_samples_per_chirp))
		{
			printf("frame format error\n");
			// ------------------------------ --------------------------------------------------
			// ofExit();
			// ------------------------------ --------------------------------------------------
		}

		// set current frame format to 64 64
		frame_format_now->num_chirps_per_frame = num_chirps;
		frame_format_now->num_samples_per_chirp = num_samples_per_chirp;
		frame_format_now->eSignalPart = (Signal_Part_t)esignalpart;
		frame_format_now->rx_mask = rx_mask;
		int32_t jj =  ep_radar_base_set_frame_format(protocolHandle,endpointRadarBase,frame_format_now);
		this->get_frame_format(protocolHandle, endpointRadarBase, frame_format_now);
		print_status_code(protocolHandle, jj);

		//get chirp duration
		int32_t chirp_duration_status = ep_radar_base_get_chirp_duration(protocolHandle, endpointRadarBase);
		print_status_code( protocolHandle, chirp_duration_status);

		// get min frame interval
		uint32_t min_frame = ep_radar_base_get_min_frame_interval(protocolHandle, endpointRadarBase);
		print_status_code( protocolHandle, min_frame);

		// distance calculations
		Device_Info_t *this_device_info = (Device_Info_t *) (device_info);
		double bandwidth_hz = (double)(this_device_info->max_rf_frequency_kHz-this_device_info->min_rf_frequency_kHz)*1000.0;
		double ramp_time_s = (double)(*chirp_duration_ns)*1e-9;
		printf("bandwidth_Hz %f\n", bandwidth_hz);
		printf("ramp_time_s %f\n", ramp_time_s);
		printf("speed_of_light %f\n", speed_of_light);

		// display some distance and related beat frequencies
		for(float this_distance=0.1; this_distance < 25; this_distance += 0.5){
			double beat_freq_hm = calculateBeatFreq(this_distance,bandwidth_hz,speed_of_light,ramp_time_s);
			printf("distance  %f (meters) \t beat_frequency %f (Hz)\n", this_distance, beat_freq_hm);
		}


		islive = true;
	}else{
		printf("init radar failed");
		islive = false; // no radar
	}

}

// ------------------------------ --------------------------------------------------
// void ofxRadar24Ghz::changeLoadData(){

// 	isloaddata = !isloaddata;
// 	if(isloaddata){
// 		if(file_loaded == false){ // remember to switch this off when file is ended
// 			ofFileDialogResult result = ofSystemLoadDialog("Load recording file");
// 			if(result.bSuccess) {
// 				string path = result.getPath();
// 				// load your file at `path`
// 				bindayDataIn.open(path.c_str(), ios::out | ios::binary);
// 				if(bindayDataIn.is_open()){
// 					file_loaded = true;
// 					ofLog(OF_LOG_NOTICE, "START loading data");
// 				}else{
// 					ofLog(OF_LOG_ERROR, "CANNOT OPEN data file");
// 					file_loaded = false;
// 				}
// 			}
// 			islive=false;

// 		}
// 	}else{
// 		if(file_loaded){
// 			bindayDataIn.close();
// 			file_loaded = false;
// 		}
// 	}
// ------------------------------ --------------------------------------------------


// }

//--------------------------------------------------------------
int ofxRadar24Ghz::radar_auto_connect() {
	// usb connections
	num_of_ports = com_get_port_list(comp_port_list, (size_t)256);
	if (num_of_ports == 0)
	{
		return -1;
	}
	else
	{
		comport = strtok(comp_port_list, delim);
		while (num_of_ports > 0)
		{
			num_of_ports--;
			// open COM port
			radar_handle = protocol_connect(comport);
			if (radar_handle >= 0)
			{
				break;
			}
			comport = strtok(NULL, delim);
		}
		return radar_handle;
	}
}

//--------------------------------------------------------------
void ofxRadar24Ghz::update() {


	// get raw data
	if(islive and !isloaddata){
		if(!acq_started){
			// start acquisition
			// enable/disable automatic trigger
			if (AUTOMATIC_DATA_FRAME_TRIGGER){
				res = ep_radar_base_set_automatic_frame_trigger(protocolHandle,
																endpointRadarBase,
																AUTOMATIC_DATA_TRIGER_TIME_US);
			}else{
				res = ep_radar_base_set_automatic_frame_trigger(protocolHandle,
																endpointRadarBase,
																0);

			}
			if(res != -1){
				acq_started = true;
			}else{
				printf("CANNOT START ACQUISITION\n");
				islive = false;
			}
		}

		res = ep_radar_base_get_frame_data(protocolHandle,	endpointRadarBase,	1);
		if(res != -1){
			// IF LIVE DATA
			for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
				for (uint32_t ANTENNA_NUMBER = 0;ANTENNA_NUMBER < (uint32_t)num_antennas ; ANTENNA_NUMBER++){
					for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < (uint32_t)num_samples_per_chirp; SAMPLE_NUMBER++){


						double this_adc_real = full_data_block[CHIRP_NUM*4*num_samples_per_chirp + (2*ANTENNA_NUMBER)*num_samples_per_chirp + SAMPLE_NUMBER]*if_scale;
						double this_adc_img  = full_data_block[CHIRP_NUM*4*num_samples_per_chirp + (2*ANTENNA_NUMBER+1)*num_samples_per_chirp + SAMPLE_NUMBER]*if_scale;
						//if (SAMPLE_NUMBER == 56 && ANTENNA_NUMBER==0){
						//	printf("%d: %f\n", CHIRP_NUM, this_adc_real);
						//}


						if(ANTENNA_NUMBER == 0){
							adc_real_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_real); // data out and scaled
							adc_imag_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_img);   // data out and scaled
							//if(SAMPLE_NUMBER == 56){
							//	printf("%d: %f\n", CHIRP_NUM, this_adc_real);
							//}
						}else if (ANTENNA_NUMBER == 1){
							adc_real_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_real); // data out and scaled
							adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_img);   // data out and scaled
						}

					}
				}
			} // chirp
			//printf("\n");
		}else{
			islive = false; // something has happed to the radar connection
		}
	}

	if(isloaddata and !islive ){
		// open file and process
		if(bindayDataIn.is_open()){
			STARTFILE:
			bindayDataIn.read((char *) full_data_block, num_antennas * num_chirps * num_samples_per_chirp * 2*sizeof(float) );
		}
		if(!bindayDataIn.good()) {
			printf("FILE ENDED");
			bindayDataIn.clear();
			bindayDataIn.seekg(0, ios::beg);
			goto STARTFILE;
		}else{
			for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
				for (uint32_t ANTENNA_NUMBER = 0;ANTENNA_NUMBER < (uint32_t)num_antennas ; ANTENNA_NUMBER++){
					for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER <(uint32_t) num_samples_per_chirp; SAMPLE_NUMBER++){

						double this_adc_real = full_data_block[CHIRP_NUM*4*num_samples_per_chirp + (2*ANTENNA_NUMBER)*num_samples_per_chirp + SAMPLE_NUMBER]*if_scale;
						double this_adc_img  = full_data_block[CHIRP_NUM*4*num_samples_per_chirp + (2*ANTENNA_NUMBER+1)*num_samples_per_chirp + SAMPLE_NUMBER]*if_scale;

						if(ANTENNA_NUMBER == 0){
							adc_real_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_real); // data out and scaled
							adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_img);   // data out and scaled
						}else if (ANTENNA_NUMBER == 1){
							adc_real_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_real); // data out and scaled
							adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = (this_adc_img);   // data out and scaled
						}

					}
				} // antenna number
			} // chirp
		}
	}

	// we got live data or we process a loaded batch from file
	if(islive or isloaddata){

		// IF RECORDING we dump data to file
		//if(isrecording){
		//	binaryDataOut.write((char *)full_data_block, num_antennas * num_chirps * num_samples_per_chirp * 2*sizeof(float));
		//}
		// MEAN REMOVAL ACROSS RANGE for RX1 and RX2
		double mean_real_tx1rx1[num_chirps];
		double mean_imag_tx1rx1[num_chirps];
		double mean_real_tx1rx2[num_chirps];
		double mean_imag_tx1rx2[num_chirps];
		// init to zero
		for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
			mean_real_tx1rx1[CHIRP_NUM] = 0.0;
			mean_imag_tx1rx1[CHIRP_NUM] = 0.0;
			mean_real_tx1rx2[CHIRP_NUM] = 0.0;
			mean_imag_tx1rx2[CHIRP_NUM] = 0.0;
		}
		for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
			for (uint32_t ANTENNA_NUMBER = 0;ANTENNA_NUMBER < (uint32_t)num_antennas ; ANTENNA_NUMBER++){
				for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER <(uint32_t) num_samples_per_chirp; SAMPLE_NUMBER++){
					if(ANTENNA_NUMBER == 0){
						mean_real_tx1rx1[CHIRP_NUM] += adc_real_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]; // data out and scaled
						mean_imag_tx1rx1[CHIRP_NUM] += adc_imag_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER];   // data out and scaled
					}else if (ANTENNA_NUMBER == 1){
						mean_real_tx1rx2[CHIRP_NUM] += adc_real_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]; // data out and scaled
						mean_imag_tx1rx2[CHIRP_NUM] += adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER];   // data out and scaled
					}
				}
			}
		}

		// put frame in complex matrix
		for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
			mean_real_tx1rx1[CHIRP_NUM] = mean_real_tx1rx1[CHIRP_NUM] / (double)(num_samples_per_chirp);
			mean_imag_tx1rx1[CHIRP_NUM] = mean_imag_tx1rx1[CHIRP_NUM] / (double)(num_samples_per_chirp);
			mean_real_tx1rx2[CHIRP_NUM] = mean_real_tx1rx2[CHIRP_NUM] / (double)(num_samples_per_chirp);
			mean_imag_tx1rx2[CHIRP_NUM] = mean_imag_tx1rx2[CHIRP_NUM] / (double)(num_samples_per_chirp);
			for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < (uint32_t)num_samples_per_chirp; SAMPLE_NUMBER++){
				double tr_1 = adc_real_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER];
				double ti_1 = adc_imag_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER];
				double tr_2 = adc_real_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER];
				double ti_2 = adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER];
				adc_real_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = tr_1 - mean_real_tx1rx1[CHIRP_NUM];
				adc_imag_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = ti_1 - mean_imag_tx1rx1[CHIRP_NUM];
				adc_real_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = tr_2 - mean_real_tx1rx2[CHIRP_NUM];
				adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER] = ti_2 - mean_imag_tx1rx2[CHIRP_NUM];
				matrix_mag_1[(CHIRP_NUM*num_chirps) + SAMPLE_NUMBER] = sqrt(pow(adc_real_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]*range_window_func[SAMPLE_NUMBER],2)+
						pow(adc_imag_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]*range_window_func[SAMPLE_NUMBER],2));
				matrix_phase_1[(CHIRP_NUM*num_chirps) + SAMPLE_NUMBER] = get_phase(adc_imag_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER],
						adc_imag_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]);
				matrix_mag_2[(CHIRP_NUM*num_chirps) + SAMPLE_NUMBER] = sqrt(pow(adc_real_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]*range_window_func[SAMPLE_NUMBER],2)+
						pow(adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]*range_window_func[SAMPLE_NUMBER],2));
				matrix_phase_2[(CHIRP_NUM*num_chirps) + SAMPLE_NUMBER] = get_phase(adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER],
						adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]);
			}
		}

		//range_fft_spectrum_hist for MTI filter enable
		double  alpha_mti = 1.0f / MTI_FILTER_LEN;
		double  beta_mti = (1.0f - alpha_mti);
		for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
			// FFT IS PADDED WITH ZERO OR TRUNCATED
			// RANGE_FFT_SIZE
			for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < RANGE_FFT_SIZE; SAMPLE_NUMBER++){
				// zero padding
				if(SAMPLE_NUMBER >= (uint32_t)num_samples_per_chirp){
					fft_1[SAMPLE_NUMBER] = (0.0);
					fft_2[SAMPLE_NUMBER] = (0.0);
				}else{
					double adc_r = adc_real_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]*range_window_func[SAMPLE_NUMBER];
					double adc_i = adc_imag_tx1rx1[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]*range_window_func[SAMPLE_NUMBER];
					fft_1[SAMPLE_NUMBER].real(adc_r);
					fft_1[SAMPLE_NUMBER].imag(adc_i);//matrix_tx1rx1[SAMPLE_NUMBER][CHIRP_NUM];
					adc_r = adc_real_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]*range_window_func[SAMPLE_NUMBER];
					adc_i = adc_imag_tx1rx2[CHIRP_NUM*num_samples_per_chirp + SAMPLE_NUMBER]*range_window_func[SAMPLE_NUMBER];
					fft_2[SAMPLE_NUMBER].real(adc_r);
					fft_2[SAMPLE_NUMBER].imag(adc_i);
				}
			}
			FFT(fft_1, RANGE_FFT_SIZE, 1);
			FFT(fft_2, RANGE_FFT_SIZE, 1);
			for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < RANGE_FFT_SIZE; SAMPLE_NUMBER++){
				rangeFFT1[SAMPLE_NUMBER][CHIRP_NUM] = fft_1[SAMPLE_NUMBER];
				rangeFFT2[SAMPLE_NUMBER][CHIRP_NUM] = fft_2[SAMPLE_NUMBER];
				//if (SAMPLE_NUMBER == 66){
				//	printf("%f, %f\n",fft_1[SAMPLE_NUMBER].real(), fft_1[SAMPLE_NUMBER].imag());
				//}
			}
			// applying MTI filtering
			if(enable_mti_filtering){
				for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < RANGE_FFT_SIZE; SAMPLE_NUMBER++){

					rangeFFT1[SAMPLE_NUMBER][CHIRP_NUM] = rangeFFT1[SAMPLE_NUMBER][CHIRP_NUM]-range_fft_spectrum_hist1[SAMPLE_NUMBER];
					rangeFFT2[SAMPLE_NUMBER][CHIRP_NUM] = rangeFFT2[SAMPLE_NUMBER][CHIRP_NUM]-range_fft_spectrum_hist2[SAMPLE_NUMBER];

					range_fft_spectrum_hist1[SAMPLE_NUMBER] = ( alpha_mti * rangeFFT1[SAMPLE_NUMBER][CHIRP_NUM]+
												 + beta_mti * range_fft_spectrum_hist1[SAMPLE_NUMBER]);
					range_fft_spectrum_hist2[SAMPLE_NUMBER] = ( alpha_mti * rangeFFT2[SAMPLE_NUMBER][CHIRP_NUM]+
												 + beta_mti * range_fft_spectrum_hist2[SAMPLE_NUMBER]);

				}
			}
		}
		// RANGE TARGET DETECTION
		// detect the targets in range by applying constant amplitude threshold over range
		// data integration of range FFT over the chirps for target range detection
		double range_tx1rx1_max[RANGE_FFT_SIZE];
		double range_tx1rx2_max[RANGE_FFT_SIZE];
		//printf("range_tx1rx2.size() %d\n", range_tx1rx2.size());
		//printf("range_tx1rx2[0].size() %d\n", range_tx1rx2[0].size());
		for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < RANGE_FFT_SIZE; SAMPLE_NUMBER++){
			double max_this_c1 = -1;
			double max_this_c2 = -1;
			for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t) num_chirps; CHIRP_NUM++){
				double	 tm1 = sqrt(pow(rangeFFT1[SAMPLE_NUMBER][CHIRP_NUM].real(),2)+
								pow(rangeFFT1[SAMPLE_NUMBER][CHIRP_NUM].imag(),2));
				double	 tm2 = sqrt(pow(rangeFFT2[SAMPLE_NUMBER][CHIRP_NUM].real(),2)+
													pow(rangeFFT2[SAMPLE_NUMBER][CHIRP_NUM].imag(),2));
				if(tm1 > max_this_c1){
					max_this_c1 = tm1;
				}
				if(tm2 > max_this_c2){
					max_this_c2 = tm2;
				}
			}
			range_tx1rx1_max[SAMPLE_NUMBER] = max_this_c1;
			range_tx1rx2_max[SAMPLE_NUMBER] = max_this_c2;
		}

		// PEAK SEARCH FIRST RESET PEAKS
		for(size_t tp=0;tp<MAX_NUM_TARGETS; tp++){
			tgt_range1[tp].index = -1;
			tgt_range1[tp].peak_val = 0.0;
			tgt_range2[tp].index = -1;
			tgt_range2[tp].peak_val = 0.0;
		}
		f_search_peak(range_tx1rx1_max, RANGE_FFT_SIZE, (double)RANGE_THRESHOLD,
				MAX_NUM_TARGETS, (double)MIN_DISTANCE,  (double)MAX_DISTANCE, (double)dist_per_bin, tgt_range1);
		// NUM PEAKS FROM ANT 1
		int num_peaks_ant_1 = 0;
		for(size_t tp=0;tp<MAX_NUM_TARGETS; tp++){
			if(tgt_range1[tp].index != -1){
				num_peaks_ant_1 += 1;
			}
		}
		f_search_peak(range_tx1rx2_max, RANGE_FFT_SIZE, (double)RANGE_THRESHOLD,
				MAX_NUM_TARGETS, (double)MIN_DISTANCE,  (double)MAX_DISTANCE, (double)dist_per_bin, tgt_range2);
		// NUM PEAKS FROM ANT 2
		int num_peaks_ant_2 = 0;
		for(size_t tp=0;tp<MAX_NUM_TARGETS; tp++){
			if(tgt_range2[tp].index != -1){
				num_peaks_ant_2 += 1;
			}
		}

		int max_t;
		int use_id=-1; // which antenna to use
		if(num_peaks_ant_1 < num_peaks_ant_2){
			max_t = num_peaks_ant_1;
			use_id = 0;
		}else{
			max_t = num_peaks_ant_2;
			use_id = 1;
		}

		// SLOW TIME PROCESSING
		// DOPPLER range_doppler_tx1rx1 [RANGE_FFT_SIZE][DOPPLER_FFT_SIZE]
		// COMPUTE MEAN ACROSS DOPPLER - only for targets i.e peaks
		vector<complex<double>> rx1_doppler_mean;
		vector<complex<double>> rx2_doppler_mean;

		if(num_peaks_ant_1 > 0 && num_peaks_ant_2 > 0){
			for(size_t tgt_=0; tgt_< (uint32_t)num_peaks_ant_1; tgt_++){
				complex<double> this_m(0.0,0.0);
				//complex<double> this_m2(0.0,0.0);
				for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
					int bin_pos = tgt_range1[tgt_].index;
					this_m += rangeFFT1[bin_pos][CHIRP_NUM];
				}
				this_m.real( this_m.real() / (double)num_chirps);
				this_m.imag( this_m.imag() / (double)num_chirps);
				rx1_doppler_mean.push_back(this_m);
				//printf("rx1_doppler_mean[%d] rx1_doppler_mean real %f \n", tgt_, rx1_doppler_mean[tgt_].real());
			}
			for(size_t tgt_=0; tgt_<(uint32_t)num_peaks_ant_2; tgt_++){
				complex<double> this_m(0.0,0.0);
				for (uint32_t CHIRP_NUM = 0; CHIRP_NUM <(uint32_t) num_chirps; CHIRP_NUM++){
					int bin_pos = tgt_range2[tgt_].index;
					this_m += rangeFFT2[bin_pos][CHIRP_NUM];
				}
				this_m.real( this_m.real() / (double)num_chirps);
				this_m.imag( this_m.imag() / (double)num_chirps);
				rx2_doppler_mean.push_back(this_m);
				//printf("rx1_doppler_mean[%d] rx1_doppler_mean real %f \n", tgt_, rx1_doppler_mean[tgt_].real());
			}

			// MEAN REMOVAL
			for(size_t tgt_=0; tgt_<(uint32_t)num_peaks_ant_1; tgt_++){
				for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
					int bin_pos = tgt_range1[tgt_].index;
					rangeFFT1[bin_pos][CHIRP_NUM] =
							rangeFFT1[bin_pos][CHIRP_NUM] - rx1_doppler_mean[tgt_];
					//printf("Chirp %d: %f %f i\n", CHIRP_NUM, fft_1[bin_pos].real(), fft_1[bin_pos].imag());
					//printf("Chirp %d: %f %f i\n", CHIRP_NUM, rx1_doppler_mean2[tgt_].real(), rx1_doppler_mean2[tgt_].imag());
					//fft_1[bin_pos] = fft_1[bin_pos] - rx1_doppler_mean2[tgt_];

				}
			}
			for(size_t tgt_=0; tgt_<(uint32_t)num_peaks_ant_2; tgt_++){
				for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
					int bin_pos = tgt_range2[tgt_].index;
					rangeFFT2[bin_pos][CHIRP_NUM] =
							rangeFFT2[bin_pos][CHIRP_NUM] - rx2_doppler_mean[tgt_];
				}
			}

			// NOW WE FILL THE RANGE DOPPLER MAP
			//range_doppler_tx1rx1
			// -------------------- RX1
			// Window for the Doppler map and prepare for FFT
			// for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < RANGE_FFT_SIZE; SAMPLE_NUMBER++){
			complex<double> range_fft_1[DOPPLER_FFT_SIZE];

			for(size_t tgt_=0; tgt_<(uint32_t)num_peaks_ant_1; tgt_++){
				for (uint32_t CHIRP_NUM = 0; CHIRP_NUM <(uint32_t) num_chirps; CHIRP_NUM++){
					int this_idx = tgt_range1[tgt_].index;
					if(this_idx < RANGE_FFT_SIZE and this_idx!= -1){
						range_fft_1[CHIRP_NUM].real(rangeFFT1[this_idx][CHIRP_NUM].real() * dopper_window_func[CHIRP_NUM]);
						range_fft_1[CHIRP_NUM].imag(rangeFFT1[this_idx][CHIRP_NUM].imag() * dopper_window_func[CHIRP_NUM]);
						//double ph = atan2(range_fft_1[CHIRP_NUM].real(), range_fft_1[CHIRP_NUM].imag());
						//if (tgt_==0){
						//	printf("%f: %f\n", range_fft_1[CHIRP_NUM].real(), range_fft_1[CHIRP_NUM].imag());
						//}

					}else{// up to DOPPLER_FFT_SIZE
						range_fft_1[CHIRP_NUM] =  (0.0);
					}
				}
				// FFT FIRST
				FFT(range_fft_1, DOPPLER_FFT_SIZE, 1);
				//if (tgt_ == 0){
				//	for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < DOPPLER_FFT_SIZE; SAMPLE_NUMBER++){
				//		fft_doppler1[SAMPLE_NUMBER] = range_fft_1[SAMPLE_NUMBER];
				//	}
				//}
				// flip before putting it in the doppler map ([half end | beg half] --> [beg half | half end])
				for(size_t THIS_DP=0; THIS_DP<DOPPLER_FFT_SIZE; THIS_DP++){
					int bin_pos = tgt_range1[tgt_].index;
					if(THIS_DP < (DOPPLER_FFT_SIZE/2)){
						range_doppler_tx1rx1[bin_pos][THIS_DP+(DOPPLER_FFT_SIZE/2)] = range_fft_1[THIS_DP];
					}else{
						range_doppler_tx1rx1[bin_pos][THIS_DP-(DOPPLER_FFT_SIZE/2)] = range_fft_1[THIS_DP];
					}
				}
			}
			// -------------------- RX2
			// Window for the Doppler map and prepare for FFT
			complex<double> range_fft_2[DOPPLER_FFT_SIZE];

			for(size_t tgt_=0; tgt_<(uint32_t)num_peaks_ant_2; tgt_++){

				for (uint32_t CHIRP_NUM = 0; CHIRP_NUM < (uint32_t)num_chirps; CHIRP_NUM++){
					int this_idx = tgt_range2[tgt_].index;
					if(this_idx < RANGE_FFT_SIZE and this_idx!= -1){
						range_fft_2[CHIRP_NUM].real(rangeFFT2[this_idx][CHIRP_NUM].real() * dopper_window_func[CHIRP_NUM]);
						range_fft_2[CHIRP_NUM].imag(rangeFFT2[this_idx][CHIRP_NUM].imag() * dopper_window_func[CHIRP_NUM]);
					}else{// up to DOPPLER_FFT_SIZE
						range_fft_2[CHIRP_NUM] =  (0.0);
					}
				}
				// FFT FIRST
				FFT(range_fft_2, DOPPLER_FFT_SIZE, 1);
				// flip before putting it in the doppler map ([half end | beg half] --> [beg half | half end])
				for(size_t THIS_DP=0; THIS_DP<DOPPLER_FFT_SIZE; THIS_DP++){
					int bin_pos = tgt_range2[tgt_].index;
					if(THIS_DP < DOPPLER_FFT_SIZE/2){
						range_doppler_tx1rx2[bin_pos][THIS_DP+(DOPPLER_FFT_SIZE/2)] = range_fft_2[THIS_DP];
					}else{
						range_doppler_tx1rx2[bin_pos][THIS_DP-(DOPPLER_FFT_SIZE/2)] = range_fft_2[THIS_DP];
					}
				}
			}

			// EXTRACTION OF INDICES FROM RANGE-DOPPLER MAP
			// TARGET MUST BE SEEN IN BOTH ANTENNAS - THIS CAN ALSO BE DIFFERENT, NO YET TARGET ID..

			int tgt_doppler_idx[max_t];
			complex<double> z1[max_t]; // Doppler range for targets -- to be used in MVDR or other super-resolution
			complex<double> z2[max_t];

			for(size_t NUM_TARGET=0; NUM_TARGET<(uint32_t)max_t; NUM_TARGET++){

				// find max val and dp index
				double max_dp=-1;
				int idx_dp = 0;
				for(size_t THIS_DP=0; THIS_DP<DOPPLER_FFT_SIZE; THIS_DP++){
					double tmp = sqrt(pow(range_doppler_tx1rx1[tgt_range1[NUM_TARGET].index][THIS_DP].real(), 2)+
							pow(range_doppler_tx1rx1[tgt_range1[NUM_TARGET].index][THIS_DP].imag(),2));
					if(max_dp<tmp){
						max_dp = tmp;
						idx_dp = THIS_DP;
					}
				}
				// consider the value of the range doppler map for the two receivers for targets with non
				// zero speed to compute angle of arrival.
				// for zero Doppler (targets with zero speed) calculate mean over Doppler to compute angle of arrival
				// index 17 corresponds to zero Doppler
				if(max_dp >= DOPPLER_THRESHOLD && idx_dp != DOPPLER_FFT_SIZE/2){
					tgt_doppler_idx[NUM_TARGET] = idx_dp;
					if(use_id == 0){
						z1[NUM_TARGET] = range_doppler_tx1rx1[tgt_range1[NUM_TARGET].index][idx_dp];//range_fft_1[tgt_range1[NUM_TARGET].index];//range_doppler_tx1rx1[tgt_range1[NUM_TARGET].index][idx_dp];
						z2[NUM_TARGET] = range_doppler_tx1rx2[tgt_range1[NUM_TARGET].index][idx_dp];//range_fft_2[tgt_range1[NUM_TARGET].index];//range_doppler_tx1rx2[tgt_range1[NUM_TARGET].index][idx_dp];
					}else if(use_id == 1){
						z1[NUM_TARGET] = range_doppler_tx1rx1[tgt_range2[NUM_TARGET].index][idx_dp];//range_fft_1[tgt_range2[NUM_TARGET].index];//range_doppler_tx1rx1[tgt_range2[NUM_TARGET].index][idx_dp];
						z2[NUM_TARGET] = range_doppler_tx1rx2[tgt_range2[NUM_TARGET].index][idx_dp];//range_fft_2[tgt_range2[NUM_TARGET].index];//range_doppler_tx1rx2[tgt_range2[NUM_TARGET].index][idx_dp];
					}
				}else{
					tgt_doppler_idx[NUM_TARGET] = DOPPLER_FFT_SIZE/2;
					z1[NUM_TARGET] = rx1_doppler_mean[NUM_TARGET];
					z2[NUM_TARGET] = rx2_doppler_mean[NUM_TARGET];
				}
			}
			// MEASUREMENT UPDATE
			//vector<Measurement> final_meas;
			for(size_t NUM_TARGET=0; NUM_TARGET<(uint32_t)max_t; NUM_TARGET++){
				 Measurement_elem_t this_measure;


				if(use_id == 0){
					this_measure.is_associated = 0;
					this_measure.strength = tgt_range1[NUM_TARGET].peak_val;
					this_measure.range = tgt_range1[NUM_TARGET].index*dist_per_bin;
					this_measure.speed = (tgt_doppler_idx[NUM_TARGET] - DOPPLER_FFT_SIZE/2) * -1.0*fD_per_bin * hz_to_mps_constant ;
					this_measure.rx1_angle_arg_re = z1[NUM_TARGET].real();
					this_measure.rx1_angle_arg_im = z1[NUM_TARGET].imag();
					this_measure.rx2_angle_arg_re = z2[NUM_TARGET].real();
					this_measure.rx2_angle_arg_im = z2[NUM_TARGET].imag();
					double this_angle = compute_angle(z1[NUM_TARGET], z2[NUM_TARGET], LAMBDA/ANTENNA_SPACING);
					this_measure.angle = this_angle;
				}else if(use_id == 1){
					this_measure.strength = tgt_range2[NUM_TARGET].peak_val;
					this_measure.range = tgt_range2[NUM_TARGET].index*dist_per_bin;
					this_measure.speed = (tgt_doppler_idx[NUM_TARGET] - DOPPLER_FFT_SIZE/2) * -1.0*fD_per_bin * hz_to_mps_constant ;
					this_measure.rx1_angle_arg_re = z1[NUM_TARGET].real();
					this_measure.rx1_angle_arg_im = z1[NUM_TARGET].imag();
					this_measure.rx2_angle_arg_re = z2[NUM_TARGET].real();
					this_measure.rx2_angle_arg_im = z2[NUM_TARGET].imag();
					double this_angle = compute_angle(z1[NUM_TARGET], z2[NUM_TARGET], LAMBDA/ANTENNA_SPACING);
					this_measure.angle = this_angle;
				}
				// Print stuff relating to targets
				if (NUM_TARGET==0){
					//printf("%f\n", this_measure.speed);
					//printf("%f\n\n", this_measure.angle);
				}
				//}---------------------------------------------------------------------------
				// RANGE IS IN METERS!!!!!
				// ANGLE IS IN DEGREES (+ CCW)!!!!
				// SPEED IS IN MS-1 (+AWAY)!!!!

				// Collect all detections (measurements)
				measurements.push_back(this_measure);
				if (!enable_tracking){
					current_targets.push_back(this_measure);
				}
			}
		}

		// Collects all measurements over n frames for better tracking (decreases time resolution of tracking)
		// put equal to 1 for normal tracking
		if (frame_id == 1){
			// Time measurement for dt in A matrix (KF)
			double current_tm2 = ros::Time::now().toSec();
			double duration_tot_sec = (current_tm2 - time_stamp);
			time_stamp = current_tm2;
		
			// tracking algorithm and avoidance controller:
			if(enable_tracking){
				//trck_lst2
				data_association2(tracking_list2, measurements, (uint16_t)max_t,
											cp_algo_settings, duration_tot_sec);
				velocity_obstacles(tracking_list2);
			}
			frame_id = 0;
			measurements.clear();
		}
		for(size_t i=0; i<MAX_NUM_TRACKS;i++){
			if (tracking_list2[i].is_alived && tracking_list2[i].measurement_counter > 5){// TODO
				double x1 = tracking_list2[i].range*100*cos((tracking_list2[i].angle)*(PI/180)) ;
				double y1 = tracking_list2[i].range*100*sin((tracking_list2[i].angle)*(PI/180)) ;
				//double current_tm = (std::chrono::duration_cast<std::chrono::microseconds>
				//(std::chrono::high_resolution_clock::now().time_since_epoch()).count())/1000.0;
				double current_tm3 = ros::Time::now().toSec();
				//printf("%f, %d, %f, %f\n", current_tm3, i, x1, y1);
			}
		}
		frame_id += 1;
	}
}

//======================================================================================================================================================
//=================================================== TRACKING ALGORITHM ===============================================================================
//======================================================================================================================================================

/**
 * Function to perform a Kalman filter to determine the velocity
 * Modifies the state Eigen::VectorXd x_hat and Eigen::MatrixXd P
 * @param x_hat vector (eigen) containing states R, th, R_dot, th_dot
 * @param y measurement vector
 * @param P covariance matrix of the filter corresponding to the track
 * NB:  radial velocity (x[2]) is negative of measurement.speed !!
 * 		angle defined positive CCW
 */
void ofxRadar24Ghz::kalmanfilter(Eigen::VectorXd& x_hat, Eigen::VectorXd& y, Eigen::MatrixXd& P){
	x_hat = A * x_hat;
	P = A*P*A.transpose() + G*G.transpose();// (Q = I) to model changing state
	Eigen::MatrixXd K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
	x_hat += K * (y - H*x_hat);
	P = (I - K*H)*P;
}

/**
 * New function for data association (Global nearest neighbor)
 * assumes that there is one track per object
 * in this function, each track will refer to a separate object
 * NB: elements in track_lst should only be considered valid if measurement_count > 3
 *
 * @param track_lst An array of type Tracking_Params_t
 * @param measurements A vector of type Measurement_elem_t
 * @param num_of_targets number of objects detected in FoV
 * @param cp_algo_settings object of algo_settings_t
*/
void ofxRadar24Ghz::data_association2(Tracking_Params_t *track_lst, vector<Measurement_elem_t>& measurements, uint16_t num_of_targets,
		 algo_settings_t *cp_algo_settings, double duration_sec){

	double KF_predict[2] = {0,0};

	// Calculate dt for A and G matrix of KF:
	Eigen::MatrixXd A(4, 4);
	Eigen::MatrixXd G(4, 2);

	A << 1,0,duration_sec,0,
		 0,1,0,duration_sec,
		 0,0,1,0,
		 0,0,0,1;
	G << 0.5*pow(duration_sec,2),0,
		 0,0.5*pow(duration_sec,2),
		 duration_sec,0,
		 0,duration_sec;
	this->A = A;
	this->G = G;//TODO bound matrices sizes

	//Check if all tracks are empty
	int8_t num_tracks = 0;
	for (uint8_t i=0; i< MAX_NUM_TRACKS; i++){
		if (track_lst[i].is_alived == 1){
			num_tracks += 1;
		//} else {// if not in use, clear:
		//	clear_track_elem(&track_lst[i]);
		}
	}
	//printf("%d, %f, %f\n", cp_algo_settings->angle_offset_deg, cp_algo_settings->wave_length_ant_spacing_ratio, cp_algo_settings->min_angle_for_track_assignment);
	// ========= CASE 1: all tracks are empty ===========

	if(num_tracks == 0 && num_of_targets != 0){
		//printf("1: %d, %d\n", num_tracks, num_of_targets);
		for (uint8_t j = 0; j < num_of_targets; j++){
			// get next empty slot in track_lst:
			uint8_t next_free_id = get_next_free_trackID2(track_lst);
			if (next_free_id == MAX_NUM_TRACKS) {// if 5 targets already being tracked
				break;
			}
			// Add initialisation to track_lst (KF init done inside):
			//printf("1\n");
			(void) init_track(track_lst, measurements[j], next_free_id, angle_correction,
									2, 50);
		
		}
	} else if (num_tracks != 0) {//========= CASE 2: update tracks already assigned ===========
		//Eigen::MatrixXd Cost = Eigen::MatrixXd::Constant(num_tracks,num_of_targets,100);
		Eigen::MatrixXd Cost = Eigen::MatrixXd::Constant(num_tracks,num_of_targets + num_tracks,100);
		int track_count = 0;
		// Creating cost matrix for GNN based on Euclidean distance:
		for (uint8_t i = 0; i < MAX_NUM_TRACKS; i++){ //for tracked target...
			if (track_lst[i].is_alived == 1){
				KF_predict[0] = (A*x_hat[i])[0];
				KF_predict[1] = (A*x_hat[i])[1];
				for (uint8_t j=0;j<num_of_targets;j++){ //for every measurement
					double r = measurements[j].range;
					double th = measurements[j].angle + angle_correction;
					// cost = d*S*d
					// Probability of misdetection dependent on angle --> do tests
					double d = sqrt(pow(r,2) + pow(KF_predict[0],2) - 2*r*KF_predict[0]*cos((th - KF_predict[1])*M_PI/180)); //Euclidean distance
					//double angle_check = th - KF_predict[1]; //TODO integrate information from IMU HERE
					// Gating: otherwise cost remains at 100
					if (d < d_abs && measurements[j].range > 0.2){
						Cost(track_count,j) = pow(d,2);
					}
				}
				// Misdetections:
				// Probability of detection P_D dependent on angle: (always bigger than 1)
				double P_D = cos(x_hat[i].coeff(1)*M_PI/180);
				Cost(track_count,num_of_targets+track_count) = 1+1-P_D;

				track_count += 1;
			}
		}
		//printf("2\n");
		// convert cost matrix into assignment matrix using Hungarian algorithm:
		vector<int> assignment;
		solve_Hungarian(Cost, assignment);
		//printf("3\n");
		// Update track with new (kalman filtered) measurements
		for (uint8_t i = 0; i < MAX_NUM_TRACKS; i++){
			if (track_lst[i].is_alived == 1){
				//printf("3.1\n");
				// If it is misdetected:
				if (assignment[i] > num_of_targets-1){
					track_lst[i].lifetime_counter += 1;
					x_hat[i] = A*x_hat[i];
					track_lst[i].range = (A*x_hat[i])[0];
					track_lst[i].angle = (A*x_hat[i])[1];
					P[i] = 10*P[i];
					//printf("3.2\n");
					// TODO how to update P matrix?

				} else {
					// Kalman filtering is done inside this function
					//printf("3.3\n");

					update_track_measure(track_lst, measurements[assignment[i]],i, angle_correction, 2);
					//printf("3.4\n");
				}

			}
		}
		//printf("4\n");
		//========= CASE 3: delete tracks ==================
		for (uint8_t i = 0; i < MAX_NUM_TRACKS; i++){
			if (track_lst[i].is_alived == 1){
				if (track_lst[i].lifetime_counter >= LIFE_TIME_COUNT || P[i].coeff(0,0) > 100){ // if misdetected (LIFE_TIME_COUNT) times, or range covariance > 100:
					track_lst[i].is_alived = 0;
					//clear_track_elem(&track_lst[i]);
					num_tracks -= 1;
					//printf("elminated\n");
				}
			}
		}
		//printf("5\n");
		//========= CASE 4: candidate track birth ===========
		// !!! Candidate tracks are still tracked in track_lst, and should be considered normal tracks when measurement_counter > GHOST_LIFE_TIME
		if (num_tracks < MAX_NUM_TRACKS){// if there are less than 5 targets being tracked
			for (uint8_t j = 0; j < num_of_targets; j++){
				if(measurements[j].is_associated == 0){// if there are still measurements over
					uint8_t next_free_id = get_next_free_trackID2( track_lst );
					if (next_free_id == MAX_NUM_TRACKS) {// if 5 targets already being tracked
						break;
					}
					// KF initialisation done inside this function:
					(void) init_track(track_lst, measurements[j], next_free_id, angle_correction,
									2, 50);//cp_algo_settings->angle_offset_deg,cp_algo_settings->wave_length_ant_spacing_ratio, cp_algo_settings->min_angle_for_track_assignment);
					
				}
			}
		}
		//printf("6\n");
	}// else if no detections or tracked objects, do nothing
	// Displaying info:
	for (uint8_t i = 0; i < MAX_NUM_TRACKS; i++){ //for tracked target...
		if (track_lst[i].is_alived == 1){
			//printf("%f\n", track_lst[i].angle);

			//printf("%f\n", P[i].coeff(3,3));
			break;
		}
	}
}


/**
 * Returns the next free slot in Tracking_Params_t[] p_tracks
 * @param p_tracks An array of type Tracking_Params_t
 * @return i Index of next free slot
 */
uint8_t ofxRadar24Ghz::get_next_free_trackID2(Tracking_Params_t *p_tracks ){
	uint8_t i = 0;

	while(i < MAX_NUM_TRACKS)	{
		if(p_tracks[i].is_alived == 0)		{
			break;
		}
		i++;
	}
	return i;
}

/**
 * Function to initiate a track:
 * fills all of the values belonging to Tracking_Params_t track_lst[id]
 * initialises KF
 *
 * @param track_lst Array of type Tracking_Params_t
 * @param measurement Pointer to Measurement_elem_t object
 * @param id integer indicating which index of track_lst to fill in
 * @return retValue =1 if initiated, =0 if not
 */
uint32_t ofxRadar24Ghz::init_track(Tracking_Params_t *track_lst, Measurement_elem_t& measurement, uint8_t id, uint16_t angle_offset_deg,
		float wave_length_ant_spacing_ratio, float min_angle){

	uint32_t retValue = 0;
	target_angle_data T;
	T = compute_angle_d(measurement.rx1_angle_arg_re, measurement.rx1_angle_arg_im,
		measurement.rx2_angle_arg_re, measurement.rx2_angle_arg_im, IGNORE_NAN, angle_offset_deg, wave_length_ant_spacing_ratio);
	
	Eigen::VectorXd x0(4);
	x0 << measurement.range,T.target_angle,-1*measurement.speed,0;
	x_hat[id] = x0;
	Eigen::MatrixXd P0(4, 4);
	int th_acc = 8;
	if (T.target_angle < 20 && T.target_angle > -20)
		th_acc = 2;
	P0 << cov_factor*0.15,0,0,0,0,cov_factor*th_acc,0,0,0,0,cov_factor*0.2,0,0,0,0,100;
	P[id] = P0;
	//printf("%f, %f\n", T.target_angle, min_angle);

	if( fabs(T.target_angle) < min_angle){
		Tracking_Params_t *p_track = &track_lst[id];

		p_track->track_id = id;
		p_track->is_alived = 1;

		p_track->measurement_counter = 1;

		p_track->range_change_flag = 0;
		p_track->speed_count = 0;

		p_track->strength = measurement.strength;
		p_track->range    = measurement.range;
		p_track->speed    = -1*measurement.speed;//TODO see if needs to be negative
		p_track->speed_th    = 0;

		p_track->rx1_angle_arg_re[0] = measurement.rx1_angle_arg_re;
		p_track->rx1_angle_arg_im[0] = measurement.rx1_angle_arg_im;
		p_track->rx2_angle_arg_re[0] = measurement.rx2_angle_arg_re;
		p_track->rx2_angle_arg_im[0] = measurement.rx2_angle_arg_im;

		p_track->angle  = T.target_angle;
		p_track->d_phi  = T.d_phi;

		// calculate the medium filter for the track angle
		median_angle_arr[id].is_full = 0;
		median_filtering(&median_angle_arr[id], p_track->angle);
		measurement.is_associated = 1;

		// Inc the number of valid tracks:zeros

		retValue = 1;
	}else{
		// Not sure if this function will do the right thing... not really necessary tho
		//clear_track_elem(&track_lst[id]);
		retValue = 0;
	}
	return retValue;
}

/**
 * Function that updates track with number id in track_lst with measurement data from data_association2
 * KF state estimation
 * 
 * @param track_lst Array of type Tracking_Params_t
 * @param measurement Pointer to Measurement_elem_t object
 * @param id integer indicating which index of track_lst to update
 */
void ofxRadar24Ghz::update_track_measure(Tracking_Params_t *track_lst, Measurement_elem_t& measurement, uint8_t id, uint16_t angle_offset_deg,
		float wave_length_ant_spacing_ratio){
	
	//printf("3.31\n");
	// Compute angle:
	target_angle_data T;
	T = compute_angle_d(measurement.rx1_angle_arg_re, measurement.rx1_angle_arg_im,
			measurement.rx2_angle_arg_re, measurement.rx2_angle_arg_im, IGNORE_NAN, angle_offset_deg, wave_length_ant_spacing_ratio);

	// Kalman filtering
	//printf("3.32\n");
	Eigen::VectorXd y(3);
	y << measurement.range,T.target_angle,-1*measurement.speed;
	// TODO make R dependent on angle
	kalmanfilter(x_hat[id],y,P[id]);
	//printf("3.33\n");

	// update values in track_lst
	Tracking_Params_t *p_track = &track_lst[id];

	// TODO check if range check flag is needed, since gating is already done...
	//float last_range = p_track->range;
	//
	p_track->strength = measurement.strength;
	p_track->range    = x_hat[id][0];
	p_track->angle    = x_hat[id][1];
	p_track->speed    = x_hat[id][2];
	p_track->speed_th = x_hat[id][3];

	p_track->rx1_angle_arg_re[0] = measurement.rx1_angle_arg_re;
	p_track->rx1_angle_arg_im[0] = measurement.rx1_angle_arg_im;
	p_track->rx2_angle_arg_re[0] = measurement.rx2_angle_arg_re;
	p_track->rx2_angle_arg_im[0] = measurement.rx2_angle_arg_im;
	// I don't know what exactly this is for...:
	if (p_track->speed == 0){
		p_track->speed_count += 1;
	}else{
		p_track->speed_count = 0;
	}

	//p_track->d_phi = T.d_phi;
	if (p_track->measurement_counter < 6){
		p_track->measurement_counter += 1;
	}
	p_track->lifetime_counter = 0;
	measurement.is_associated = 1;
}


/**
 * Function to convert cost matrix into an assignment matrix
 * Modifies the input matrix Cost
 *
 * note: this uses the copyright code .... TODO
 *
 * @param Cost An Eigen matrix with cost of association as values
 */
void ofxRadar24Ghz::solve_Hungarian(Eigen::MatrixXd& Cost, vector<int>& assignment){

	uint8_t n = Cost.rows();
	uint8_t m = Cost.cols();
	vector<vector<double>> costMatrix;
	for (uint8_t i=0; i<n;i++){
		vector<double> row_n;
		for (uint8_t j=0; j<m;j++){
			row_n.push_back(Cost.coeff(i,j));
		}
		costMatrix.push_back(row_n);
	}

	HungarianAlgorithm HungAlgo;
	(void) HungAlgo.Solve(costMatrix, assignment);
}

//======================================================================================================================================================
//=================================================== VELOCITY OBSTACLES ===========================================================================
//======================================================================================================================================================

/**
 * Function to perform velocity obstacles of NEAREST OBJECT
 * finds set of velocity vectors that will result in a collision
 * if true_VO == false: will output avoidance direction opposite to bearing of object
 * if true_VO == true: will output desired avoidance velocity vector
 */
void ofxRadar24Ghz::velocity_obstacles(Tracking_Params_t *track_lst){

	// for every target, find the collision cone (min and max direction vectors of velocity that will result in a collision)
	float r_a, r_b, margin, R, th_cc;
	vector<float> th_min, th_max, weight;
	r_a = 0.15;// radius of the drone
	r_b = 0.25;// radius of the obstacle poles
	margin = 0.2;
	R = r_a + r_b + margin;
	double avoidance_margin = 0.1;
	double weight_init = 100;
	uint8_t track_index = 0;
	uint8_t avoid_state_temp = 0;
	double v_xa_des_temp = 0;
	double v_ya_des_temp = 0;
	for (uint8_t i=0; i< MAX_NUM_TRACKS; i++){
		if (track_lst[i].is_alived == 1 && track_lst[i].measurement_counter > 5){//TODO tune this param
			th_cc = asin(R/track_lst[i].range);
			th_min.push_back(track_lst[i].angle*M_PI/180 - th_cc);
			th_max.push_back(track_lst[i].angle*M_PI/180 + th_cc);
			weight.push_back(track_lst[i].range);


			// NOTE !!! Angle in this definition is defined positive CCW !!!
			// velocity direction of self w.r.t. object (V_AB)
			//                      <---------tangential velocity----------><-radial velocity->
			float direction = atan2(track_lst[i].speed_th*track_lst[i].range,track_lst[i].speed) + track_lst[i].angle*M_PI/180 + M_PI;
			float mag = sqrt(pow(track_lst[i].speed_th*track_lst[i].range, 2) + pow(track_lst[i].speed,2));
			if (direction > M_PI) {
				direction -= 2*M_PI;
			} else if (direction < -M_PI){
				direction += 2*M_PI;
			}
			//printf("%f, %f, %f\n", th_min[i],th_max[i] , direction);
			bool condition = false;
			if (track_lst[i].range/track_lst[i].speed < 2) {// if time to contact is less than 2s
				condition = true;
			}
			if (weight[track_index] < weight_init) {// if closest obstacle:
				if (direction > th_min[track_index] && direction < th_max[track_index] && condition) {//if relative velocity vector is within collision cone
					//find V_B = V_A - V_AB:
					double v_xb = this->v_xa - mag*cos(direction);
					double v_yb = this->v_ya - mag*sin(direction);
					double desired_direction = 0;
					// if obstacle to the right, avoid right: OR //if (abs(direction - th_min[track_index]) > abs(direction - th_max[track_index])) {
					// note: accuracy of direction in CC is not very accurate, so more predictable implementation:
					if (abs(direction - th_min[track_index]) < abs(direction - th_max[track_index])) {//(track_lst[i].angle < 0) {
						avoid_state_temp = 1;//right
						desired_direction = th_min[track_index] - avoidance_margin;
					} else {
						avoid_state_temp = -1;//left
						desired_direction = th_max[track_index] + avoidance_margin;
					}
					v_xa_des_temp = v_xb + mag*cos(desired_direction);
					v_ya_des_temp = v_yb + mag*sin(desired_direction);
					double mag_ratio = 0.5/sqrt(pow(v_xa_des_temp, 2) + pow(v_ya_des_temp,2));// normalising factor
					v_xa_des_temp = v_xa_des_temp * mag_ratio;
					v_ya_des_temp = v_ya_des_temp * mag_ratio;
					//printf("%f, %f, %f, %f, %f, %f\n", this->v_xa, this->v_ya, v_xb, v_yb, v_xa_des_temp, v_ya_des_temp);
				} else {
					avoid_state_temp = 0;
				}
				weight_init = weight[track_index];
			}
			//if (this->true_VO) {
			//	avoid_state_temp = avoid_state_temp*-1;
			//}
			double n_dir = atan2(v_ya_des_temp, v_xa_des_temp);
			if (isnan(n_dir)) {
				this->avoid_state = 1;
			    this->v_xa_des = 0;
			    this->v_ya_des = 0.5;
			} else {
				this->avoid_state = avoid_state_temp;
				this->v_xa_des = v_xa_des_temp;
				this->v_ya_des = v_ya_des_temp;
			}
			if (this->avoid_state != 0) {
				//printf("n_dir: %f\n", n_dir*180/M_PI);
			}
			track_index += 1;
		}
	}
}

// ---------------------------------------------
double ofxRadar24Ghz::get_phase(float real, float imag){
	double phi;

	/* Phase angle (0 to 2Pi) */
	if((real > 0) && (imag >= 0))		// 1st quadrant
	{
		phi = atan((double)imag / (double)real);
	}
	else if((real < 0) && (imag >= 0))	// 2nd quadrant
	{
		phi = atan((double)imag / (double)real) + PI;
	}
	else if((real < 0) && (imag <= 0)) 	// 3rd quadrant
	{
		phi = atan((double)imag / (double)real) + PI;
	}
	else if((real > 0) && (imag <= 0)) 	// 4th quadrant
	{
		phi = atan((double)imag / (double)real) + 2*PI;
	}
	else if((real == 0) && (imag > 0))
	{
		phi = PI/2;
	}
	else if((real == 0) && (imag < 0))
	{
		phi = 3*PI/2;
	}
	else
	{
		phi = 0;	// Indeterminate
	}

	return(phi);
}

// ---------------------------------------------
target_angle_data ofxRadar24Ghz::compute_angle_d(float if1_i, float if1_q, float if2_i, float if2_q,
		double d_old, int16_t angle_offset_deg, float wave_length_ant_spacing_ratio)
{

	//float wave_length_ant_spacing_ratio = (TGT_WAVE_LENGTH_MM / TGT_ANTENNA_SPACING_MM);

	target_angle_data temp;

	float rx1_ang, rx2_ang;

	float d_phi;

	float delta_angle;

	float target_angle;

	rx1_ang = get_phase(if1_i, if1_q); //- (double)(0.13 * PI);

	rx2_ang = get_phase(if2_i, if2_q);

	d_phi = (rx1_ang - rx2_ang);

	if (d_phi <= 0)
	{
		d_phi += 2*PI;
	}
	d_phi -= PI;

	if ((uint32_t)d_old == IGNORE_NAN)
	{
		target_angle = 0;
	}
	else if (d_phi > d_old + 0.9* PI || d_phi < d_old - 0.9* PI)
	{
	   d_phi = d_old;
	}

	/* Arcus sinus (-PI/2 to PI/2), input= -1..1 */
	target_angle = asin(d_phi * wave_length_ant_spacing_ratio / (2*PI));

	target_angle = target_angle * 180 / PI;	// Angle (-90...90°)

	target_angle = target_angle + (double)((int32_t) angle_offset_deg + ANGLE_QUANTIZATION * 0.5);

	delta_angle  = fmodf(target_angle , (double)ANGLE_QUANTIZATION);

	target_angle -= delta_angle;

	temp.d_phi = d_phi;

	temp.target_angle = target_angle;

	return temp;
}

//===========================================================================
float ofxRadar24Ghz::median_filtering(Median_Filtering_t *track_median_arr, float new_input){

	if( track_median_arr->median_filter_len > MAX_MEDIAN_FILTER_LEN)
		track_median_arr->median_filter_len = MAX_MEDIAN_FILTER_LEN;

	if (track_median_arr->is_full == 0)
	{
		for (uint32_t j = 0; j < track_median_arr->median_filter_len; j++)
		{
			track_median_arr->buffer[j] = new_input;
		}

		track_median_arr->is_full = 1;


		return new_input;
	}
	else
	{
		float sorting_arr[MAX_MEDIAN_FILTER_LEN];

		uint32_t len = MAX_MEDIAN_FILTER_LEN-1;
		if( len > track_median_arr->median_filter_len-1)
			len = track_median_arr->median_filter_len-1;

		for (uint32_t j = 0; j < len; j++)
		{

			sorting_arr[j] = track_median_arr->buffer[j+1];		// shift the array left in order to add new value

			track_median_arr->buffer[j] = track_median_arr->buffer[j+1];

		}


		track_median_arr->buffer[track_median_arr->median_filter_len-1] = new_input;
		sorting_arr[track_median_arr->median_filter_len-1] = new_input;
		qsort(sorting_arr, track_median_arr->median_filter_len, sizeof(float), compare_float);
		return sorting_arr[track_median_arr->median_filter_len / 2];
	}
}

//===========================================================================
int ofxRadar24Ghz::compare_float(const void *a, const void *b){
	int retval = 0;

	float a_f = *(float*)a;
	float b_f = *(float*)b;

	if (a_f > b_f)
	{
		retval = 1;
	}
	else if (a_f < b_f)
	{
		retval = -1;
	}

	return retval;
}

//===========================================================================
void ofxRadar24Ghz::f_search_peak(double *fft_spectrum, int search_lenght, double threshold,
		int max_target_count, double min_distance,  double max_ditance, double dist_per_bin, target_peak * peak_idx ){

	// search for peaks
	int peak_cnt = 0;
	for(size_t n=3; n < (uint32_t)search_lenght-3; n++){
		//printf("searching for peaks n = %d\n", n);

		int fp_bin = n;
		int f1_bin = fp_bin -1;
		int f12_bin  = fp_bin -2;
		int fr_bin = fp_bin +1;
		int fr2_bin = fp_bin +2;

		double fp = fft_spectrum[fp_bin];
		double f1 = fft_spectrum[f1_bin];
		double f12 = fft_spectrum[f12_bin];
		double fr = fft_spectrum[fr_bin];
		double fr2 = fft_spectrum[fr2_bin];

		float peak_idxs = 0;
		uint32_t target_range;
		if(fp >= threshold && fp>= f12 && fp>=f1 && fp > fr && fp > fr2){

		    peak_idxs = (f12_bin * f12 + f1_bin * f1 + fp_bin * fp + fr_bin * fr + fr2_bin * fr2) / (f12 + f1 + fp + fr + fr2);
		    target_range = (uint32_t)((peak_idxs -1) * dist_per_bin);
			//double curr_range = (double) (fp_bin -1) * dist_per_bin;

			if(target_range >= min_distance && target_range <= max_ditance){

		        float fp_new;

		        if(peak_idxs > fp_bin)
		          fp_new = fp +(fr - fp) * (peak_idxs - fp_bin) / (fr_bin - fp_bin);
		        else
		          fp_new= f1 + (fp - f1) * (fp_bin - peak_idxs) / (fp_bin - f1_bin);

		        //target_info[num_of_targets].strength = fp_new;  // FFT magnitude level
		        //target_info[num_of_targets].range = target_range; // Range in centimeters (cm)
		        //target_info[num_of_targets].idx = (uint32_t)(peak_idxs + 0.5);  // index of FFT where target is detected (rounded)
		        //num_of_targets++;

		        peak_idx[peak_cnt].index = peak_idxs;
		        peak_idx[peak_cnt].peak_val = fp_new;
		        peak_cnt+=1;
				if(peak_cnt>=max_target_count){
					return;
					break;
				}
			}
		}
	}
	return;
}

//===========================================================================
void ofxRadar24Ghz::clearTargets(){
	current_targets.clear();
	for(size_t i=0; i<MAX_NUM_TARGETS;i++){
		for(size_t TARGET=0; TARGET < tracking_list[i].num_of_tracks; TARGET++){
			tracking_list[i].num_of_tracks = 0;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////
// CALLBACKS
/* 24Ghz radar USB
 * Helper function for the ep_radar_base_get_frame_data
 * called every time ep_radar_base_get_frame_data method
 * is called to return measured time domain signals
 * */

//===========================================================================
void ofxRadar24Ghz::print_status_code( int32_t protocol_handle, int32_t status){

	//check status
	const char * hr_current_frame_format = protocol_get_status_code_description(protocol_handle, status);
	char buffer [1500];
	//int n;
	sprintf(buffer, "%s", hr_current_frame_format);
	printf("[%s]\n",buffer);


}

//===========================================================================
void ofxRadar24Ghz::get_frame_format( int32_t protocol_handle,
        uint8_t endpoint,
        Frame_Format_t* frame_format){

	// query
	int32_t current_frame_format = ep_radar_base_get_frame_format(protocolHandle, endpointRadarBase);
	print_status_code(protocolHandle, current_frame_format);

	// cast and read data format
	Frame_Format_t * frame_format_disp = (Frame_Format_t *) (frame_format_current);
	printf("num_chirps_per_frame %d\n", frame_format_disp->num_chirps_per_frame);
	printf("num_samples_per_chirp %d\n", frame_format_disp->num_samples_per_chirp);
	printf("rx_mask %d\n", frame_format_disp->rx_mask);
	printf("ONLY_I = 0 /  ONLY_Q = 1 / I_AND_Q = 2 %d\n", frame_format_disp->eSignalPart);

	frame_format->num_chirps_per_frame = frame_format_disp->num_chirps_per_frame;
	frame_format->num_samples_per_chirp = frame_format_disp->num_samples_per_chirp;
	frame_format->rx_mask = frame_format_disp->rx_mask;
	frame_format->eSignalPart = frame_format_disp->eSignalPart;

}

//===========================================================================
void ofxRadar24Ghz::received_frame_data(void* context,
						int32_t protocol_handle,
		                uint8_t endpoint,
						const Frame_Info_t* frame_info){

    float *full_data_block = (float *) (context);
    int num_ant = 2;
    if(frame_info->rx_mask == 3){
    	num_ant = 2;
    }

	/*printf("RFM frame_number %d\n", frame_info->frame_number); //12
	printf("RFM num_chirps %d\n", frame_info->num_chirps);
	printf("RFM num_rx_antennas %d\n", frame_info->num_rx_antennas);
	printf("RFM num_samples_per_chirp %d\n", frame_info->num_samples_per_chirp);
	printf("RFM rx_mask %d\n", frame_info->rx_mask);
	printf("RFM interleaved_rx %d\n", frame_info->interleaved_rx);
	printf("RFM data_format %d\n", frame_info->data_format);*/
    /*frame_start = &frame_info->sample_data[CHIRP_NUMBER *
                                  num_rx_antennas *
                                  num_samples_per_chirp *
                                ((data_format == RADAR_RX_DATA_REAL)? 1 : 2)];*/

    /* data_value_real = frame_start[(2 * ANTENNA_NUMBER    ) *
                                   num_samples_per_chirp + SAMPLE_NUMBER];
    * data_value_imag = frame_start[(2 * ANTENNA_NUMBER + 1) *
    *                               num_samples_per_chirp + SAMPLE_NUMBER];*/


	for (uint32_t ANTENNA_NUMBER = 0; ANTENNA_NUMBER < (uint32_t)num_ant ; ANTENNA_NUMBER++){
		//uint32_t start = ant*frame_info->num_chirps*frame_info->num_samples_per_chirp*1
		for (uint32_t CHIRP_NUMBER = 0;CHIRP_NUMBER <  frame_info->num_chirps; CHIRP_NUMBER++){
			for (uint32_t SAMPLE_NUMBER = 0; SAMPLE_NUMBER < frame_info->num_samples_per_chirp; SAMPLE_NUMBER++)
			{
				if(frame_info->data_format != 0){
					const float * frame_start =  &frame_info->sample_data[CHIRP_NUMBER*num_ant*frame_info->num_samples_per_chirp*2];

					full_data_block[CHIRP_NUMBER*4*frame_info->num_samples_per_chirp + (2*ANTENNA_NUMBER)*frame_info->num_samples_per_chirp + SAMPLE_NUMBER] =
												frame_start[(2*ANTENNA_NUMBER)*frame_info->num_samples_per_chirp+SAMPLE_NUMBER];

					full_data_block[CHIRP_NUMBER*4*frame_info->num_samples_per_chirp + (2*ANTENNA_NUMBER+1)*frame_info->num_samples_per_chirp + SAMPLE_NUMBER] =
												frame_start[(2*ANTENNA_NUMBER+1)*frame_info->num_samples_per_chirp+SAMPLE_NUMBER];

				}else{
					printf("Not implemented: data format is real.. please check format.");
				}
			}
		}
	}
}

/* Function to get no of set bits in binary
   representation of positive integer n */
//===========================================================================
int ofxRadar24Ghz::countSetBits(unsigned int n){
	unsigned int count = 0;
	while (n) {
		count += n & 1;
		n >>= 1;
	}
	return count;
}

//===========================================================================
void ofxRadar24Ghz::received_temperature(void* context,
		int32_t protocol_handle,
        uint8_t endpoint,
		uint8_t temp_sensor,
        int32_t temperature_001C){

	//
    //float *temperature = (float *) (context);
    //printf("temperature %d:\n", frame_info->num_temp_sensors);

}

//===========================================================================
void ofxRadar24Ghz::received_frame_format(void* context,
		int32_t protocol_handle,
        uint8_t endpoint,
        const Frame_Format_t* frame_format){

	Frame_Format_t *frame_format_current = (Frame_Format_t *) (context);

	/*printf("num_chirps_per_frame %d\n", frame_format->num_chirps_per_frame);
	printf("num_samples_per_chirp %d\n", frame_format->num_samples_per_chirp);
	printf("rx_mask %d\n", frame_format->rx_mask);
	printf("ONLY_I = 0 /  ONLY_Q = 1 / I_AND_Q = 2 %d\n", frame_format->eSignalPart);*/

	frame_format_current->num_chirps_per_frame = frame_format->num_chirps_per_frame;
	frame_format_current->num_samples_per_chirp = frame_format->num_samples_per_chirp;
	frame_format_current->rx_mask = frame_format->rx_mask;
	frame_format_current->eSignalPart = frame_format->eSignalPart;


	//printf("data format is %d", frame_format->data_format);
	/*
	EP_RADAR_BASE_SIGNAL_ONLY_I  = 0,< Only the I signal is captured
                                           during radar data frame
                                           acquisition.
    EP_RADAR_BASE_SIGNAL_ONLY_Q  = 1, < Only the Q signal is captured
                                           during radar data frame
                                           acquisition.
    EP_RADAR_BASE_SIGNAL_I_AND_Q = 2  < Both, I and Q signal are captured
                                           as a complex signal during radar
                                           data frame acquisition. */

}

//===========================================================================
void ofxRadar24Ghz::get_device_info(void* context,
        int32_t protocol_handle,
        uint8_t endpoint,
		const Device_Info_t * device_info){

	Device_Info_t *this_device_info = (Device_Info_t *) (context);

	this_device_info->description = device_info->description;
	this_device_info->min_rf_frequency_kHz = device_info->min_rf_frequency_kHz;
	this_device_info->max_rf_frequency_kHz = device_info->max_rf_frequency_kHz;
	this_device_info->num_tx_antennas = device_info->num_tx_antennas;
	this_device_info->num_rx_antennas = device_info->num_rx_antennas;
	this_device_info->max_tx_power = device_info->max_tx_power;
	this_device_info->num_temp_sensors = device_info->num_temp_sensors;
	this_device_info->major_version_hw = device_info->major_version_hw;
	this_device_info->minor_version_hw = device_info->minor_version_hw;
	this_device_info->interleaved_rx = device_info->interleaved_rx;
	this_device_info->data_format = device_info->data_format;

	printf("max_tx_power %d\n", device_info->max_tx_power);
	printf("num_tx_antennas %d\n", device_info->num_tx_antennas);
	printf("num_rx_antennas %d\n", device_info->num_rx_antennas);
	printf("data_format %d interleaved_rx %d\n", device_info->data_format, device_info->interleaved_rx);
	printf("min_rf_frequency_kHz %d max_rf_frequency_kHz %d\n", device_info->min_rf_frequency_kHz, device_info->max_rf_frequency_kHz);
	printf("bandwidth %d kHz\n", device_info->max_rf_frequency_kHz-device_info->min_rf_frequency_kHz);
	printf("num_temp_sensors  %d\n", device_info->num_temp_sensors);
	printf("version %d-%d\n", device_info->major_version_hw, device_info->minor_version_hw);

}

/* * \param[in] context          The context data pointer, provided along with
 *                             the callback itself through
 *                             \ref ep_radar_base_set_callback_tx_power.
 * \param[in] protocol_handle  The handle of the connection, the sending
 *                             device is connected to.
 * \param[in] endpoint         The number of the endpoint that has sent the
 *                             message.
 * \param[in] tx_antenna       The number of the TX antenna from which the
 *                             power was measured.
 * \param[in] tx_power_001dBm  The power value in 0.001 dBm.*/
//===========================================================================
void ofxRadar24Ghz::get_tx_power(void* context,
			int32_t protocol_handle,
			uint8_t endpoint,
			uint8_t tx_antenna,
			int32_t tx_power_001dBm){

	uint32_t * power_set = (uint32_t *) context;
	*power_set = tx_power_001dBm;
	printf("power is set to %f dBm\n", (double)tx_power_001dBm*(1e-3));


}

//===========================================================================
void ofxRadar24Ghz::set_fmcw_conf(void* context,
            int32_t protocol_handle,
            uint8_t endpoint,
            const Fmcw_Configuration_t*
              fmcw_configuration){

	//Fmcw_Configuration_t *sd = (Fmcw_Configuration_t * )context;
	printf("lower_frequency_kHz %d \n", fmcw_configuration->lower_frequency_kHz);
	printf("upper_frequency_kHz %d \n", fmcw_configuration->upper_frequency_kHz);
	printf("tx_power %d \n", fmcw_configuration->tx_power);


}

//===========================================================================
void ofxRadar24Ghz::get_bw_sec(void* context,
		   int32_t protocol_handle,
		   uint8_t endpoint,
		   uint32_t bandwidth_per_second){

	uint32_t * bps = (uint32_t *) context;
	printf("bandwidth_per_second %d \n", bandwidth_per_second);
	* bps = bandwidth_per_second;

}

//===========================================================================
void ofxRadar24Ghz::get_chirp_duration(void* context,
        int32_t protocol_handle,
        uint8_t endpoint,
        uint32_t chirp_duration_ns){

	uint32_t * cd = (uint32_t *) context;
	printf("chirp Duration is %d ns\n", chirp_duration_ns);
	*cd = chirp_duration_ns;
}

//===========================================================================
void ofxRadar24Ghz::get_min_frame_interval(void* context,
        int32_t protocol_handle,
        uint8_t endpoint,
        uint32_t min_frame_interval_us){

	uint32_t * cd = (uint32_t *) context;
	printf("min_frame_interval is %d us\n", min_frame_interval_us);
	*cd = min_frame_interval_us;

}

// --------------------------------------------------------------
/**
 * to map a range of values to RGB color gradients
 */
//===========================================================================
void ofxRadar24Ghz::getcolor(int value, int norm_p, float * r, float * g, float * b) {

	    float minimum = 0;
	    float maximum = norm_p;
	    float ratio = 2 * (value-minimum) / (maximum - minimum);

	    float tmp_a = 255.0*(1.0 - ratio);
	    if(tmp_a > 0){
	    	*b = tmp_a;
	    }else{
	    	*b = 0.0f;
	    }
	    float tmp_b = 255.0*(ratio - 1);
	    if(tmp_b > 0){
	    	*r =tmp_b;
	    }else{
	    	*r = 0.0;
	    }
	    *g = 255.0 - *b - *r;

}
