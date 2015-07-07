%% This code was developed as part of graduate coursework at Embry-Riddle Aeronautical University
%% under the guidance of Dr. Pat Anderson and Dr. Hever Moncayo.

%% Author: Robert F. Hartley
%% Assistant Developers: Francois Hugon, Brian DeRosa, and Christopher Carvalho
%% Support: apm2.simulink@gmail.com

%% Oct. 25, 2012
%% Version 1.0


ProjDir = pwd;

cd([ProjDir '/blocks/Serial/']);
mex sfunar_serialPrintFloats.c;
mex sfunar_serialRead.c;
mex sfunar_serialWrite.c;

cd([ProjDir '/blocks/Timing/']);
mex Arduino_Clock_sfcn.c;
mex Arduino_RealTimeMonitor_sfcn.c;


cd([ProjDir '/blocks/Analog_IO/']);
mex sfunar_analogOutput.c;
mex sfunar_analogInput.c;

cd([ProjDir '/blocks/RCChannels/']);
mex Arduino_RCRead_sfcn.c;
mex Arduino_RCWrite_sfcn.c;

cd([ProjDir '/blocks/Magnetometer/']);
mex Arduino_Mag_sfcn.c;

cd([ProjDir '/blocks/GPS/']);
mex Arduino_GPS_sfcn.c;

cd([ProjDir '/blocks/Pitot/']);
mex Arduino_Pitot_sfcn.c;

cd([ProjDir '/blocks/IMU/']);
mex Arduino_IMU_sfcn.c;

% cd([ProjDir '/blocks/EEPROM/']);
% mex Arduino_EEPROMRead_sfcn.c;

cd([ProjDir '/blocks/Discrete_IO/']);
mex sfunar_digitalOutput.c;
mex sfunar_digitalInput.c;

% cd([ProjDir '/blocks/Auxiliary/']);
% mex s2b.c s2b_wrapper.c;

cd([ProjDir '/blocks/Baro/']);
mex Arduino_Baro_sfcn.c;

cd([ProjDir '/blocks/FlashMemory/']);
mex Arduino_DataFlash_WriteFloats.c;

cd(ProjDir);


