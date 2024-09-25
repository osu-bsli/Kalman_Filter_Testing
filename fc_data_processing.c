/*
 * fc_data_processing.c
 *
 *  Created on: Nov 12, 2023
 *      Author: bsli
 */

#include "fc_data_processing.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

FILE *CSV_in, *CSV_out;
char *CSV_out_name = "data/flight_data_processed.csv", *CSV_in_name = "data/flight_data_2.csv";

const char* getfield(char* line, int num) {
    const char* tok;
    for (tok = strtok(line, ",");
            tok && *tok;
            tok = strtok(NULL, ",\n"))
    {
        if (!--num)
            return tok;
    }
    return NULL;
}

int main() {
  // Create data structs
  struct fc_unprocessed_data unprocessed_data; 
  struct fc_processed_data processed_data;
  EKF filter;
  float P[2] = {0.1,0.1}, Q[2] = {0.001,0.001}, R[3] = {0.011,0.011,0.011};

  // Initialize data
  fc_init_data(&processed_data);
  EKF_Init(&filter, P, Q, R, 0.0f, LAUNCH_PITCH);

  // TODO: open CSV files
  CSV_in = fopen(CSV_in_name, "r+");
  CSV_out = fopen(CSV_out_name, "w+");
  fprintf(CSV_out, "time,velocity,vertical_velocity,altitude,vertical_velocity_2,altitude_2,air_density,pitch,temperature,phi,theta,baro_height\n");

  // Call process_sensor_data
  fc_process_sensor_data(&filter, &processed_data, &unprocessed_data);

  // Close CSV files
  fclose(CSV_in);
  fclose(CSV_out);
  return 1;
}

int fc_init_data(struct fc_processed_data *processed_data) {
	processed_data->velocity = 0.0;
	processed_data->velocity_vertical = 0.0;

	processed_data->altitude = 0.0;

  processed_data->velocity_vertical_2 = 0.0;

	processed_data->altitude_2 = 0.0;
	processed_data->air_density = PAD_AIR_DENSITY;

	processed_data->pitch = LAUNCH_PITCH;
  processed_data->phi = 0.0f;
  processed_data->theta = LAUNCH_PITCH;
	processed_data->acceleration = G;

	processed_data->temperature = PAD_AIR_TEMP;



	processed_data->stage = pad;
	processed_data->current_airbrake_angle = 0.0;
	processed_data->current_drag = 0.298407;

	processed_data->last_time = 0.0;
}

int fc_process_sensor_data(EKF *filter, struct fc_processed_data *processed_data, struct fc_unprocessed_data *unprocessed_data) {
  printf("Processing . . . ");
  // Loop through CSV rows
  char line[1024];
  fgets(line, 1024, CSV_in);

  while (fgets(line, 1024, CSV_in)) {
    //printf("%s\n\n\n", line);
    // Read sensor data into a struct
    fc_read_sensor_data(unprocessed_data, line);
    // Process data
    fc_estimate_state(filter, processed_data, unprocessed_data);

    // Write processed data to CSV output
    fprintf(CSV_out,"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", unprocessed_data->current_time, processed_data->velocity, processed_data->velocity_vertical, processed_data->altitude, processed_data->velocity_vertical_2, processed_data->altitude_2, processed_data->air_density, processed_data->pitch, processed_data->temperature, processed_data->phi, processed_data->theta, unprocessed_data->baro_height);
  }
  printf("Complete.\n");
  return 1;
}

int fc_read_sensor_data(struct fc_unprocessed_data *data, char line[]) {
  // Dupe string
  char* tmp = strdup(line);

  // Load data
  data->accelerometer_x = atof(getfield(tmp, 2));
  tmp = strdup(line);
  data->accelerometer_y = atof(getfield(tmp, 3));
  tmp = strdup(line);
  data->accelerometer_z = atof(getfield(tmp, 4));
  tmp = strdup(line);

  data->magne_x = atof(getfield(tmp, 5));
  tmp = strdup(line);
  data->magne_y = atof(getfield(tmp, 6));
  tmp = strdup(line);
  data->magne_z = atof(getfield(tmp, 7));
  tmp = strdup(line);
  
  data->bmi_accel_z = atof(getfield(tmp, 11));//x
  tmp = strdup(line);
  data->bmi_accel_x = -1*atof(getfield(tmp, 12));//y
  tmp = strdup(line);
  data->bmi_accel_y = -1*atof(getfield(tmp, 13));//z
  tmp = strdup(line);
  data->bmi_gyro_z = atof(getfield(tmp, 8));//x
  tmp = strdup(line);
  data->bmi_gyro_x = -1*atof(getfield(tmp, 9));//y
  tmp = strdup(line);
  data->bmi_gyro_y = -1*atof(getfield(tmp, 10));//z
  tmp = strdup(line);
  data->magne_x = atof(getfield(tmp, 5));
  tmp = strdup(line);
  data->magne_y = atof(getfield(tmp, 6));
  tmp = strdup(line);
  data->magne_z = atof(getfield(tmp, 7));
  tmp = strdup(line);

  data->baro_height = atof(getfield(tmp, 16))/3.281;
  tmp = strdup(line);
  data->current_time = atof(getfield(tmp, 1));
  tmp = strdup(line);



  // Free dupe
  free(tmp);
}

int fc_estimate_state(EKF *filter, struct fc_processed_data *processed_data, struct fc_unprocessed_data *unprocessed_data) {
  // Calculate altitude from barometer
  float baro_height = unprocessed_data->baro_height;

  // //~~~~~~~ Kalman filter to estimate pitch~~~~~~~~
  // // Predict
  float T = (unprocessed_data->current_time - processed_data->last_time);
  processed_data->last_time = unprocessed_data->current_time;
  // float p = unprocessed_data->bmi_gyro_x;
  // float q = unprocessed_data->bmi_gyro_y;
  // float r = unprocessed_data->bmi_gyro_z;
  // float sp = sin(processed_data->phi);
  // float cp = cos(processed_data->phi);
  // float tt = tan(processed_data->theta);
  // processed_data->phi = processed_data->phi + T*(p + tt*(q*sp + r*cp));
  // processed_data->theta = processed_data->theta + T*(q*cp - r*sp);
  // sp = sin(processed_data->phi);
  // cp = cos(processed_data->phi);
  // float st = sin(processed_data->theta);
  // float ct = cos(processed_data->theta);
  // float A[4] = {tt*(q*cp - r*sp), (r*cp + q*sp)*(tt*tt + 1.0), -(r*cp + q*sp), 0.0};
  // float Ptmp[4] = {T*(Q[0] + 2.0*A[0]*P[0] + A[1]*P[2]), T*(A[0]*P[1] + A[2]*P[0] + A[1]*P[3] + A[3]*P[1]), T*(A[0]*P[2] + A[2]*P[0] + A[1]*P[3] + A[3]*P[2]), T*(Q[1] + A[2]*P[2] + 2.0*A[3]*P[3])};
  // P[0] = P[0] + Ptmp[0];
  // P[1] = P[1] + Ptmp[1];
  // P[2] = P[2] + Ptmp[2];
  // P[3] = P[3] + Ptmp[3];
  // // Update
  // float ax = unprocessed_data->accelerometer_y;
  // float ay = unprocessed_data->accelerometer_x;
  // float az = unprocessed_data->accelerometer_z;
  // float h[3] = { G*st, -G*ct*sp, -G*ct*cp};
  // float C[6] = {0.0, G*ct, -G*ct, G*sp*st, G*sp*ct, G*cp*st};
  // float tempInv[9];
  // tempInv[0] = P[3]*C[1]*C[1] + R[0];
  // tempInv[1] = C[1]*C[2]*P[2] + C[1]*C[3]*P[3];
  // tempInv[2] = C[1]*C[4]*P[2] + C[1]*C[5]*P[3];
  // tempInv[3] = C[1]*(C[2]*P[1] + C[3]*P[3]);
  // tempInv[4] = R[1] + C[2]*(C[2]*P[0] + C[3]*P[2]) + C[3]*(C[2]*P[1]+C[3]*P[3]);
  // tempInv[5] = C[4]*(C[2]*P[0] + C[3]*P[2]) + C[5]*(C[2]*P[1] + C[3]*P[3]);
  // tempInv[6] = C[1]*(C[4]*P[1] + C[5]*P[3]);
  // tempInv[7] = C[2]*(C[4]*P[0]) + C[5]*(C[4]*P[1] + C[5]*P[3]);
  // tempInv[8] = R[2] + C[4]*(C[4]*P[0] + C[5]*P[2])+ C[5]*(C[4]*P[1] + C[5]*P[3]);
  // float tempInvdetinv = 1.0/(tempInv[0]*tempInv[4]*tempInv[8] - tempInv[0]*tempInv[5]*tempInv[7] - tempInv[1]*tempInv[3]*tempInv[8] + tempInv[1]*tempInv[5]*tempInv[6] + tempInv[2]*tempInv[3]*tempInv[7] - tempInv[2]*tempInv[4]*tempInv[6]);
  // float tempInvinv[9];
  // tempInvinv[0] = tempInvdetinv*(tempInv[4]*tempInv[8] - tempInv[5]*tempInv[7]);
  // tempInvinv[1] = -tempInvdetinv*(tempInv[1]*tempInv[8] - tempInv[2]*tempInv[7]);
  // tempInvinv[2] = tempInvdetinv*(tempInv[1]*tempInv[5] - tempInv[2]*tempInv[4]);
  // tempInvinv[3] = -tempInvdetinv*(tempInv[3]*tempInv[8] - tempInv[5]*tempInv[6]);
  // tempInvinv[4] = tempInvdetinv*(tempInv[0]*tempInv[8] - tempInv[2]*tempInv[6]);
  // tempInvinv[5] = -tempInvdetinv*(tempInv[0]*tempInv[5] - tempInv[2]*tempInv[3]);
  // tempInvinv[6] = tempInvdetinv*(tempInv[3]*tempInv[7] - tempInv[4]*tempInv[6]);
  // tempInvinv[7] = -tempInvdetinv*(tempInv[0]*tempInv[7] - tempInv[1]*tempInv[6]);
  // tempInvinv[8] = tempInvdetinv*(tempInv[0]*tempInv[4] - tempInv[1]*tempInv[3]);
  // float K[6] = {tempInvinv[3]*(C[2]*P[0] + C[3]*P[1]) + tempInvinv[6]*(C[4]*P[0] + C[5]*P[1]) + C[1]*tempInvinv[0]*P[1], tempInvinv[4]*(C[2]*P[0] + C[3]*P[1]) + tempInvinv[7]*(C[4]*P[0] + C[5]*P[1]) + C[1]*tempInvinv[1]*P[1], tempInvinv[5]*(C[2]*P[0] + C[3]*P[1]) + tempInvinv[8]*(C[4]*P[0] + C[5]*P[1]) + C[1]*tempInvinv[2]*P[1],
  // 	  	  	  	tempInvinv[3]*(C[2]*P[2] + C[3]*P[3]) + tempInvinv[6]*(C[4]*P[2] + C[5]*P[3]) + C[1]*tempInvinv[0]*P[3], tempInvinv[4]*(C[2]*P[2] + C[3]*P[3]) + tempInvinv[7]*(C[4]*P[2] + C[5]*P[3]) + C[1]*tempInvinv[1]*P[3], tempInvinv[5]*(C[2]*P[2] + C[3]*P[3]) + tempInvinv[8]*(C[4]*P[2] + C[5]*P[3]) + C[1]*tempInvinv[2]*P[3]};
  // Ptmp[0] = -P[2]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - P[0]*(C[2]*K[1] + C[4]*K[2] - 1.0);
  // Ptmp[1] = -P[3]*(C[1]*K[0] + C[3]*K[1] + C[5]*K[2]) - P[1]*(C[2]*K[1] + C[4]*K[2]) - P[1]*(C[2]*K[1] + C[4]*K[2] - 1.0);
  // Ptmp[2] = -P[2]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5] - 1.0) - P[0]*(C[2]*K[4] + C[4]*K[5]);
  // Ptmp[3] = -P[3]*(C[1]*K[3] + C[3]*K[4] + C[5]*K[5]) - P[1]*(C[2]*K[4] + C[4]*K[5] - 1.0) - P[1]*(C[2]*K[4] + C[4]*K[5]);
  // P[0] = P[0] + Ptmp[0];
  // P[1] = P[1] + Ptmp[1];
  // P[2] = P[2] + Ptmp[2];
  // P[3] = P[3] + Ptmp[3];
  // processed_data->phi = processed_data->phi + K[0]*(ax - h[0]) + K[1]*(ay - h[1]) + K[2]*(ax - h[2]);
  // processed_data->theta = processed_data->theta + K[3]*(ax - h[0]) + K[4]*(ay - h[1]) + K[5]*(ax - h[2]);
  // processed_data->pitch = processed_data->theta;

  /*~~~ Phil's Pitch Kalman Filter ~~~*/
  // Predict
  EKF_Predict(filter, unprocessed_data->bmi_gyro_x, unprocessed_data->bmi_gyro_y, unprocessed_data->bmi_gyro_z, T);
  // Update
  EKF_Update(filter, unprocessed_data->bmi_accel_x, unprocessed_data->bmi_accel_y, unprocessed_data->bmi_accel_z);
  processed_data->phi = filter->phi_r;
  processed_data->pitch = filter->theta_r;
  processed_data->theta = filter->theta_r;
  if (unprocessed_data->current_time < 10.1 && unprocessed_data->current_time > 9.9) {
    printf("  %f,%f,%f,%f   %f,%f   %f,%f,%f   ", filter->P[0][0], filter->P[0][1], filter->P[1][0], filter->P[1][1], filter->Q[0], filter->Q[1], filter->R[0], filter->R[1], filter->R[2]);
  }
  //Calculate acceleration from accelerometer
  float ax = unprocessed_data->accelerometer_x;
  float ay = unprocessed_data->accelerometer_y;
  float az = unprocessed_data->accelerometer_z;
  processed_data->acceleration = sqrt(ax*ax + ay*ay + az*az);

  /**
  // Complementary filter to estimate vertical velocity
  processed_data->velocity_vertical = COMP_ALPHA*(processed_data->altitude + next_alt)/T + (1 - COMP_ALPHA)*(processed_data->velocity_vertical - T*processed_data->acceleration*sin(processed_data->theta));
  processed_data->altitude = next_alt;
  */

  /*~~~ Kalman filter to estimate vertical velocity and altitude ~~~*/
  float m11 = 1;
  float m21 = 0;
  float m12 = 0;
  float m22 = 0;
  float var_acc = 1.0; // TODO: Adjust

  float Q11 = var_acc*0.25*(T*T*T*T);
  float Q12 = var_acc*0.5*(T*T*T);
  float Q21 = var_acc*0.5*(T*T*T);
  float Q22 = var_acc*(T*T);

  float R11 = 0.008; // TODO: Adjust originally 0.008

  float ps1,ps2,opt;
  float pp11,pp12,pp21,pp22;
  float inn, ic, kg1, kg2;

  ps1 = processed_data->altitude + T*processed_data->velocity_vertical;
  ps2 = processed_data->velocity_vertical;

  opt = T*m22;
  pp12 = m12 + opt + Q12;

  pp21 = m22 + Q22;
  pp11 = m11 + T*(m12 + pp21) + Q11;
  pp21 += Q21;
  pp22 = m22 + Q22;

  inn = baro_height - ps1;
  ic = pp11 + R11;

  kg1 = pp11/ic;
  kg2 = pp21/ic;

  processed_data->altitude = ps1 + kg1*inn;
  processed_data->velocity_vertical = ps2 + kg2*inn;

  opt = 1 - kg1;
  m11 = pp11*opt;
  m12 = pp12*opt;
  m21 = pp21 - pp11*kg2;
  m22 = pp22 - pp12*kg2;

  // Calculate velocity
  processed_data->velocity = processed_data->velocity_vertical/sin(processed_data->pitch);

  return 1;
}
