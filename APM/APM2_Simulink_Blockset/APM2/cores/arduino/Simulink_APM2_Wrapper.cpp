#ifndef MATLAB_MEX_FILE

#include "Arduino.h"
#include <FastSerial.h>
#include <inttypes.h>
#include <string.h>
#include <AP_Common.h>
#include <AP_Compass.h>
#include <AP_Math.h>
#include <I2C.h>
#include <SPI.h>
#include <AP_PeriodicProcess.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_Compass_HMC5843.h> /* this library supports HMC5883L */
#include <AP_GPS.h>
#include <AP_Baro.h> 
#include <AP_InertialSensor.h>
#include <AP_IMU.h>
#include <APM_RC.h>
#include <Pitot.h>
#include <TimeStats.h>
#include <rtwtypes.h>
#include <stdbool.h> /* definition of boolean */
#include <DataLogger.h>
#include <string.h>
#include "startup_menu.h"



/* Preprocessors used by DataFlash */
#define BLOCK_FORMAT_HEAD_BYTE1 0xF1  // Block format packet byte 1
#define BLOCK_FORMAT_HEAD_BYTE2 0xF2  // Block format packet byte 2
#define BLOCK_FORMAT_TAIL_BYTE  0xF3  // Block format packet tail byte
#define HEAD_BYTE1              0x46  // Data packet byte 1
#define HEAD_BYTE2              0x57  // Data packet byte 2
#define TAIL_BYTE               0x56  // Data packet tail byte


/* Macro to create serial ports */
FastSerialPort0(Serial);
FastSerialPort1(Serial1); 
FastSerialPort2(Serial2); /* UART2 == Serial2. For TM, solder bridge in place */

/* Declare instance of these classes for use */
Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  scheduler;
AP_Baro_MS5611 baro;
AP_InertialSensor_MPU6000 ins( 53 ); /* chip select is pin 53 */
AP_IMU_INS imu(&ins);
AP_GPS_MTK16 gps(&Serial1);
AP_Compass_HMC5843 compass;
I2C eye2c;
APM_RC_APM2 APM_RC;
Pitot pitot;
TimeStats timestats;
/*DataFlash_APM2  DataFlash;*/
DataLogger datalog(&Serial);

/* Data Logger */
extern "C" void datalog_register_block(const char* name, int32_t len, int32_t precision) {datalog.register_block(name,len,precision);}
extern "C" void datalog_write_float_array(float* data, uint32_t len) {datalog.write_float_array(data,len);}
extern "C" bool datalog_check_for_data(void) {return datalog.check_for_data();}
extern "C" bool datalog_is_writeable(void) {return datalog.is_writeable();}
extern "C" void datalog_enable_write(void) {datalog.enable_write();}
extern "C" void datalog_disable_write(void) {datalog.disable_write();}
extern "C" void datalog_erase(void) {datalog.erase();}
extern "C" void datalog_dump_to_serial(FastSerial* ps) {datalog.dump_to_serial(ps);}
extern "C" void datalog_init(void) {datalog.init();}
extern "C" void datalog_clear_file_header(void) {datalog.clear_file_header();}

/* Magnetometer */
extern "C" void    compass_init(void)                                       { compass.init(); }
extern "C" void    compass_set_offsets(int16_t x, int16_t y, int16_t z)     { compass.set_offsets(x,y,z); }
extern "C" void    compass_set_initial_location(int32_t lat, int32_t lon)   { compass.set_initial_location(lat, lon); }
extern "C" void    compass_set_declination(float rad)                       { compass.set_declination(rad); }
extern "C" uint8_t compass_read(void)                                       { return (uint8_t)compass.read(); }
extern "C" float   compass_calculate_heading(float roll, float pitch)       { return compass.calculate_heading(roll,pitch); }
extern "C" int16_t compass_get_product_id(void) { return compass.product_id; }     
extern "C" int16_t compass_get_healthy(void)    { return compass.healthy; }
extern "C" int16_t compass_get_mag_x(void)      { return compass.mag_x; }   
extern "C" int16_t compass_get_mag_y(void)      { return compass.mag_y; }
extern "C" int16_t compass_get_mag_z(void)      { return compass.mag_z; }

/* GPS */
extern "C" void    gps_init(void)  { gps.init(); }
extern "C" void    gps_update(void)  { gps.update(); }
extern "C" uint8_t gps_status(void) { return gps.status(); }
extern "C" uint8_t  gps_get_new_data(void)       { return (uint8_t)gps.new_data; }     
extern "C" int32_t  gps_get_latitude(void)       { return gps.latitude; }
extern "C" int32_t  gps_get_longitude(void)      { return gps.longitude; }
extern "C" int32_t  gps_get_altitude(void)       { return gps.altitude; }
extern "C" int32_t  gps_get_ground_speed(void)   { return gps.ground_speed; }
extern "C" int32_t  gps_get_ground_course(void)  { return gps.ground_course; }
extern "C" uint8_t  gps_get_num_sats(void)       { return gps.num_sats; }
extern "C" uint32_t gps_get_time(void)           { return gps.time; }
extern "C" uint32_t gps_get_date(void)           { return gps.date; }
extern "C" uint16_t gps_get_hdop(void)           { return gps.hdop; }
extern "C" void     gps_set_new_data(uint8_t c)  { gps.new_data = c; }  

/* I2C */
extern "C" void     I2C_begin(void) { eye2c.begin(); }
extern "C" uint8_t  I2C_write(uint8_t c0, uint8_t c1, uint8_t c2) { eye2c.write(c0,c1,c2); }
extern "C" uint8_t  I2C_read(uint8_t c0, uint8_t c1, uint8_t c2) { eye2c.read(c0,c1,c2); }
extern "C" uint8_t  I2C_receive(void) { eye2c.receive(); }
extern "C" void     I2C_timeOut(uint16_t to) {eye2c.timeOut(to); }

/* Scheduler */
extern "C" void    scheduler_init(void)                 { scheduler.init(&isr_registry); }
extern "C" void    isr_registry_init(void)              { isr_registry.init(); }

/* Baro */
extern "C" void    baro_init(void)                      { baro.init(&scheduler); }
extern "C" void    baro_calibrate(void)                 { baro.calibrate(delay); } 
extern "C" void    baro_read(void)                      { baro.read(); }
extern "C" float   baro_get_pressure(void)              { return baro.get_pressure(); }
extern "C" float   baro_get_temperature(void)           { return baro.get_temperature(); }

/* IMU Uncalibrated */
/*
extern "C" void    ins_init(void)                       { ins.init(&scheduler); }
extern "C" void    ins_update(void)                     { ins.update(); }
extern "C" void    ins_get_gyros(float * imu_gyro)      { ins.get_gyros(imu_gyro); }
extern "C" void    ins_get_accels(float * imu_accel)    { ins.get_accels(imu_accel); }
extern "C" float   ins_temperature(void)                { return ins.temperature(); }
*/

/* Inertial Measurement Unit (calibrated) */

extern "C" void        imu_flash_leds(bool on) 
{
    digitalWrite(27, on?HIGH:LOW); 
    digitalWrite(25, on?LOW:HIGH);
}

extern "C" void        imu_init(uint32_t lpf_filt_freq_hz, uint32_t gyro_scale, uint32_t accel_scale)           {imu.init(IMU::COLD_START, delay, imu_flash_leds, &scheduler,lpf_filt_freq_hz, gyro_scale, accel_scale);};
extern "C" void        imu_update(void)        {imu.update();}
extern "C" void        imu_get_accels(float* a) {Vector3f accel; accel = imu.get_accel(); a[0]=accel.x; a[1]=accel.y; a[2]=accel.z;}
extern "C" void        imu_get_gyros(float* g)  {Vector3f gyro; gyro = imu.get_gyro(); g[0]=gyro.x; g[1]=gyro.y; g[2]=gyro.z;}
extern "C" float       imu_temperature(void)                { return ins.temperature(); } 


/* Serial for comms */
extern "C" int      Serial_begin(long r) { Serial.begin(r); }
extern "C" int      Serial_read(void) { return Serial.read(); }
extern "C" void     Serial_end(void) { Serial.end(); }
extern "C" void     Serial_write(uint8_t * c, size_t s) { Serial.write(c, s); }
extern "C" int      Serial_available(void) {return Serial.available();}
extern "C" void     Serial_print(const char* s) {Serial.print(s);}
extern "C" void     Serial_println(const char* s) {Serial.println(s);}
extern "C" void     Serial_printfloats(double * d, size_t s)
{
    uint16_t i;
    uint16_t len;
    
    len = (uint16_t)s;
    
    for (i=0;i<len;i++)
    {
        Serial.print(d[i]);
        Serial.print(" ");
    }
    
    Serial.print("\n");
}

/* Serial for GPS */
extern "C" int      Serial1_begin(long r) { Serial1.begin(r); }

/* Serial for Telemetry: SOLDER BRIDGE IN PLACE */
extern "C" int      Serial2_begin(long r) { Serial2.begin(r); }
extern "C" int      Serial2_read(void) { return Serial2.read(); }
extern "C" void     Serial2_write(uint8_t * c, size_t s) { Serial2.write(c, s); }
extern "C" void     Serial2_end(void) { Serial2.end(); }
extern "C" int      Serial2_available(void) {return Serial2.available();}
extern "C" void     Serial2_print(const char* s) {Serial2.print(s);}
extern "C" void     Serial2_println(const char* s) {Serial2.println(s);}
extern "C" void     Serial2_printfloats(double * d, size_t s)
{
    uint16_t i;
    uint16_t len;
//     char pfbuffer[20];
    
    len = (uint16_t)s;
    
    for (i=0;i<len;i++)
    {
//       (void)sprintf(pfbuffer, "%12.12f", 3.14159);
//       Serial2.print(pfbuffer);
        Serial2.print(d[i]);
      Serial2.print(" ");
    }
    
    Serial2.print("\n");
}

/* Timing stats */
extern "C" void        timestats_mark(void)                { timestats.mark(); }
extern "C" uint32_t    timestats_get_delta_ms(void)        { timestats.get_delta_ms(); }
extern "C" uint32_t    timestats_get_delta_us(void)        { timestats.get_delta_us(); }
extern "C" void        timestats_set_specified_frame_rate_ms(uint32_t ts) {timestats.set_specified_frame_rate_ms(ts);}
extern "C" uint32_t    timestats_get_specified_frame_rate_ms(void) { return timestats.get_specified_frame_rate_ms();}
extern "C" uint32_t    timestats_get_millis(void)        { timestats.get_millis(); }

/* SPI */
extern "C" void     SPI_begin(void) { SPI.begin(); }
extern "C" void     SPI_setClockDivider(uint8_t d) { SPI.setClockDivider(d); }

/* RC Channels */
extern "C" void     APM_RC_Init(void)                               { APM_RC.Init(&isr_registry); }
extern "C" void     APM_RC_enable_out(uint8_t c)                    { APM_RC.enable_out(c); }
extern "C" uint8_t  APM_RC_GetState(void)                           { return APM_RC.GetState(); }
extern "C" uint16_t APM_RC_InputCh(uint8_t i)                       { return APM_RC.InputCh(i); }
extern "C" void     APM_RC_OutputCh(uint8_t ch, uint16_t pwm)       { APM_RC.OutputCh(ch,pwm); }

/* Pitot probe */
extern "C" void     pitot_calibrate(void)               { pitot.calibrate(); }
extern "C" float    pitot_get_dynamic_pressure(void)    { return pitot.get_dynamic_pressure(); }

/* System start up */
extern "C" void Simulink_APM2_Startup(void)
{
  char c, d;
  bool done = false;
  bool haveuser = false;
  uint8_t numlogs;
  int lastlog, laststartpage, lastendpage;
  float percent_mem_remaining;
  bool exit_menu = false;

  /* Start up serial bus. Note GPS serial is started up by the GPS block because only it uses that bus. */
  /* Serial_begin(115200); */ /* Make this variable from serial block with default here?? */
  
  /* Start up I2C bus */
  I2C_begin();
  I2C_timeOut(20);
  
  /* Start up SPI bus */
  SPI_begin();
  SPI_setClockDivider(0x01); /* SPI_CLOCK_DIV32 0x06, 500khz for debugging, increase later */ /* SPI_CLOCK_DIV16 0x01 */
  
  /* Start up scheduler */
  isr_registry_init();
  scheduler_init();
  
  /* Give the user options to deal with data upon start up. Since this uses serial busses, it is placed here
   * so as to not interfere with other bus sharing blocks.
   */
  datalog_init();

  /* Start up serial busses and say hello */
Serial_begin(115200);
  print_greeting(&Serial);

  if (datalog_check_for_data())
    {
      while (!exit_menu)
	{
	  /* make this parallel soon */
	  c = get_data_request(&Serial);
	  
	  /* do something with response */
	  switch (c)
	    {
	    case ANS_ERASE_MEMORY:
	      datalog_enable_write();
	      datalog_erase();
	      datalog_disable_write();
	      exit_menu = true;
	      break;
	    case ANS_PRINT_TO_TERMINAL:
	      datalog_dump_to_serial(&Serial);
	      break;
	    case ANS_OVERWRITE:
	      datalog_enable_write();
	      datalog_clear_file_header();
	      datalog_disable_write();
	      exit_menu = true;
	      break;
	    case ANS_LEAVE_ALONE:
	    case ANS_NO_RESPONSE:
	      exit_menu = true;
	    }
	}
    }

  exit_menu = false;
  if (!datalog_check_for_data())
    {
      while (!exit_menu)
	{
	  /* make this parallel soon */
	  c = get_no_data_found_request(&Serial);
      
	  /* do something with response */
	  switch (c)
	    {
	    case ANS_ERASE_MEMORY:
	      datalog_enable_write();
	      datalog_erase();
	      datalog_disable_write();
	      break;
	    case ANS_ARM_DATA_REC:
	      datalog_enable_write();
	      exit_menu = true;
	      break;
	    case ANS_LEAVE_ALONE:
	    case ANS_NO_RESPONSE:
	      exit_menu = true;
	    }
	}
    }
  

  print_goodbye(&Serial);
  delay(1000); // give serial time to actually send the message before killing bus
  //Serial_end();
  //Serial2_end();


// The below commented code displays the flash data interaction menu via UART2 (Serial2 - Telemetry)
  // Serial2_begin(115200);
//   print_greeting(&Serial2);
// 
//   if (datalog_check_for_data())
//     {
//       while (!exit_menu)
// 	{
// 	  /* make this parallel soon */
// 	  c = get_data_request(&Serial2);
// 	  
// 	  /* do something with response */
// 	  switch (c)
// 	    {
// 	    case ANS_ERASE_MEMORY:
// 	      datalog_enable_write();
// 	      datalog_erase();
// 	      datalog_disable_write();
// 	      exit_menu = true;
// 	      break;
// 	    case ANS_PRINT_TO_TERMINAL:
// 	      datalog_dump_to_serial(&Serial2);
// 	      break;
// 	    case ANS_OVERWRITE:
// 	      datalog_enable_write();
// 	      datalog_clear_file_header();
// 	      datalog_disable_write();
// 	      exit_menu = true;
// 	      break;
// 	    case ANS_LEAVE_ALONE:
// 	    case ANS_NO_RESPONSE:
// 	      exit_menu = true;
// 	    }
// 	}
//     }
// 
//   exit_menu = false;
//   if (!datalog_check_for_data())
//     {
//       while (!exit_menu)
// 	{
// 	  /* make this parallel soon */
// 	  c = get_no_data_found_request(&Serial2);
//       
// 	  /* do something with response */
// 	  switch (c)
// 	    {
// 	    case ANS_ERASE_MEMORY:
// 	      datalog_enable_write();
// 	      datalog_erase();
// 	      datalog_disable_write();
// 	      break;
// 	    case ANS_ARM_DATA_REC:
// 	      datalog_enable_write();
// 	      exit_menu = true;
// 	      break;
// 	    case ANS_LEAVE_ALONE:
// 	    case ANS_NO_RESPONSE:
// 	      exit_menu = true;
// 	    }
// 	}
//     }
//   
// 
//   print_goodbye(&Serial2);
//   delay(1000); // give serial time to actually send the message before killing bus
//   //Serial_end();
//   //Serial2_end();



  /* Start up baro sensor now that SPI has been started */
  baro_init();
  baro_calibrate();
  
  
  /* Start up RC channels */
  APM_RC_Init();
  APM_RC_enable_out((uint8_T)0);
  APM_RC_enable_out((uint8_T)1);
  APM_RC_enable_out((uint8_T)2);
  APM_RC_enable_out((uint8_T)3);
  APM_RC_enable_out((uint8_T)4);
  APM_RC_enable_out((uint8_T)5);
  APM_RC_enable_out((uint8_T)6);
  APM_RC_enable_out((uint8_T)7);
  
  /* Start up pitot probe using our own class */
  pinMode(0, INPUT); /* Should already be an input but JIC */
  pitot_calibrate();

}


#endif /* #ifndef MATLAB_MEX_FILE */
