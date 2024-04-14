/*
 * fc_data_processing.h
 *
 *  Created on: Nov 12, 2023
 *      Author: bsli
 */

#ifndef INC_FC_DATA_PROCESSING_H_
#define INC_FC_DATA_PROCESSING_H_

#include <time.h>

#define LAUNCH_PITCH 	1.5708
#define PAD_AIR_DENSITY	1.225
#define PAD_AIR_TEMP	32.0
#define G				9.81
#define COMP_ALPHA		0.5

enum flight_stage {
	pre_pad = 0,
	pad = 1,
	boost = 2,
	coast = 3,
	control = 4,
	drogue = 5,
	main_chute = 6,
	landed = 7,
};

struct fc_processed_data {
	// Sensor State
	float velocity;
	float velocity_vertical;

	float altitude;
	float air_density;

	float pitch; // radians
	float acceleration;

	float temperature;

	// Control State
	float current_airbrake_angle;
	float current_cross_sectional_area;
	float current_drag;
	enum flight_stage stage;

	float phi;
	float theta;
	clock_t last_tick;
};

struct fc_unprocessed_data {
	float accelerometer_x;
	float accelerometer_y;
	float accelerometer_z;

	float magne_x;
	float magne_y;
	float magne_z;

	float bmi_accel_x;
	float bmi_accel_y;
	float bmi_accel_z;
	float bmi_gyro_x;
	float bmi_gyro_y;
	float bmi_gyro_z;

	float air_pressure_mbar;
	float air_temperature_C;
	float baro_height;
};

float P[4] = {0.1,0.1,0.1,0.1};
float Q[4] = {0.001,0.001,0.001,0.001};
float R[4] = {0.011,0.011,0.011,0.011};

/* Functions */
int fc_init_data_processing(struct fc_processed_data *processed_data, struct fc_unprocessed_data *unprocessed_data);
int fc_process_sensor_data(struct fc_processed_data *processed_data, struct fc_unprocessed_data *unprocessed_data);
int fc_read_sensor_data(struct fc_unprocessed_data *data);
int fc_estimate_state(struct fc_processed_data *processed_data, struct fc_unprocessed_data *unporcessed_data);




#endif /* INC_FC_DATA_PROCESSING_H_ */
