#ifndef SIMULINK_APM2_WRAPPER_H
#define SIMULINK_APM2_WRAPPER_H

/* Only include things here if they are needed for type definitions. The other includes should go in the .cpp file */
#include <Arduino.h>
#include <inttypes.h>
#include <stdio.h> /* for size_t */
#include <string.h>
#include <stdbool.h> /* definition of boolean */


/* Data Logger */
void datalog_register_block(const char* name, int32_t len, int32_t precision);
void datalog_write_float_array(float* data, int32_t len);
bool datalog_check_for_data(void);
bool datalog_is_writeable(void);
void datalog_enable_write(void);
void datalog_disable_write(void);
void datalog_erase(void);
void datalog_dump_to_serial(void* ps);
void datalog_init(void);
void datalog_clear_file_header(void);

/*
void datalog_init(void);
void datalog_dump_to_serial(void* ps, uint8_t options); 
bool datalog_check_for_data(void);
void datalog_register_block(const char* name, uint8_t type, uint32_t len, int8_t precision);
void datalog_write_bytes(uint8_t* data, uint32_t len);
void datalog_enable_write(void);
void datalog_erase(void);
bool datalog_is_empty(void);
void datalog_clear_file_header(void);
*/

/*
void    DataFlash_Init(void);
void    DataFlash_NeedErase(void);
void    DataFlash_EraseAll(void (*delay_cb)(unsigned long));
void    DataFlash_start_new_log(void);
int     DataFlash_find_last_log(void);
void    DataFlash_get_log_boundaries(uint8_t log_num, \
				     int start_page, \
				     int end_page);
uint8_t DataFlash_get_num_logs(void);
void    DataFlash_StartWrite(int16_t PageAdr);
void    DataFlash_FinishWrite(void);
void    DataFlash_WriteFloat(float data);
void    DataFlash_StartRead(int16_t PageAdr);
float   DataFlash_ReadFloat(void);
void    DataFlash_WriteFloatArray(float* fdata, uint32_t len);
void    DataFlash_WriteBlockData(const char* colnames, uint8_t len, uint8_t precision);
*/

/* Magnetometer */
/* wrappers */
void    compass_init(void);
void    compass_set_offsets(int16_t x, int16_t y, int16_t z);
void    compass_set_initial_location(int32_t lat, int32_t lon);
void    compass_set_declination(float rad);
uint8_t compass_read(void);
float   compass_calculate_heading(float roll, float pitch);

/* added functions to get class element */
int16_t compass_get_product_id(void);
int16_t compass_get_healthy(void);
int16_t compass_get_mag_x(void);
int16_t compass_get_mag_y(void);
int16_t compass_get_mag_z(void);

/* I2C */
void I2C_begin(void);
uint8_t I2C_write(uint8_t c0, uint8_t c1, uint8_t c2);
uint8_t I2C_read(uint8_t c0, uint8_t c1, uint8_t c2);
uint8_t I2C_receive(void);
void    I2C_timeOut(uint16_t to);


/* GPS */
/* wrappers */
void    gps_init(void);
void    gps_update(void);
uint8_t gps_status(void);

/* added functions to get/set class elements */
uint8_t     gps_get_new_data(void);
void        gps_set_new_data(uint8_t);
int32_t     gps_get_latitude(void);
int32_t     gps_get_longitude(void);
int32_t     gps_get_altitude(void);
int32_t     gps_get_ground_speed(void);
int32_t     gps_get_ground_course(void);
int16_t     gps_get_hdop(void);
uint8_t     gps_get_num_sats(void);
uint32_t    gps_get_time(void);
uint32_t    gps_get_date(void);

/* Scheduler Functions */
void    scheduler_init(void);
void    isr_registry_init(void);

/* Baro sensor */
void    baro_init(void);
void    baro_calibrate(void);
void    baro_read(void);
float   baro_get_pressure(void); /* mbar*100 */
float   baro_get_temperature(void); /* degC*100 */

/* Inertial sensor (uncalibrated) */
/*
void    ins_init(void);
void    ins_init(void);
void    ins_update(void);
void    ins_get_gyros(float * imu_gryo);
void    ins_get_accels(float * imu_accels);
float   ins_temperature(void);
*/

/* Inertial Measurement Unit (calibrated) */
void        imu_init(uint32_t, uint32_t, uint32_t);
void        imu_flash_leds(bool); 
void        imu_update(void);
void        imu_get_accels(float*);
void        imu_get_gyros(float*);
float       imu_temperature(void);

/* Serial */
int     Serial_begin(long r);
void    Serial_end(void);
int     Serial_read(void);
void    Serial_write(uint8_t * c, size_t s);
int     Serial_available(void);
void    Serial_print(const char* s);
void    Serial_println(const char* s);
int     Serial1_begin(long r);

int     Serial2_begin(long r);
int     Serial2_read(void);
void    Serial2_end(void);
void    Serial2_write(uint8_t * c, size_t s);
void    Serial2_available(void);
void    Serial2_print(const char* s);
void    Serial2_println(const char* s);
void    Serial_printfloats(double * d, size_t s);
void    Serial2_printfloats(double * d, size_t s);

/* SPI */
void     SPI_begin(void);
void     SPI_setClockDivider(uint8_t d);

/* RC Channels */
void     APM_RC_Init(void);
void     APM_RC_enable_out(uint8_t c);
uint8_t  APM_RC_GetState(void);
uint16_t APM_RC_InputCh(uint8_t i);
void     APM_RC_OutputCh(uint8_t ch, uint16_t pwm);


void     pitot_calibrate(void);
float    pitot_get_dynamic_pressure(void);

/* System start up */
void Simulink_APM2_Startup(void);

/* Timing stats */
void        timestats_mark(void);
uint32_t    timestats_get_delta_ms(void);
uint32_t    timestats_get_delta_us(void);
void        timestats_set_specified_frame_rate_ms(uint32_t ts);
uint32_t    timestats_get_specified_frame_rate_ms(void);
uint32_t    timestats_get_millis(void);

#endif /* Simulink_APM2_Wrapper.h */

