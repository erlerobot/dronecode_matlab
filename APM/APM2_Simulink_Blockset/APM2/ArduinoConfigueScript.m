%% This code was developed as part of graduate coursework at Embry-Riddle Aeronautical University
%% under the guidance of Dr. Pat Anderson and Dr. Hever Moncayo.

%% Author: Robert F. Hartley
%% Assistant Developers: Francois Hugon, Brian DeRosa, and Christopher Carvalho
%% Support: apm2.simulink@gmail.com

%% Oct. 25, 2012
%% Version 1.0


%% Define the location of the files
% ProjDir = pwd; % Edit this for appropriate file locations
ProjDir = '/home/hartley/cyberdyne/Arduino_Code/ArduPilot/APM2';
OperatingSystem = 'ubuntu1204';

%% Arduino Setup script
disp('Setting Arduino preferences');
switch (OperatingSystem)
    case 'windows'
        arduino.Prefs.setArduinoPath([ProjDir,'\Arduino\IDE\arduino-0022']);
    case 'ubuntu1204'
        arduino.Prefs.setArduinoPath('/usr/share/arduino');
end


%arduino.Prefs.setBoard('mega'); % For Arduino Mega 1280
arduino.Prefs.setBoard('mega2560'); % For Arduino Mega 2560
try
    comPorts=arduino.Prefs.searchForComPort;
    arduino.Prefs.setComPort(comPorts{1});
catch
    disp('Failed to find/set com port for Arduino board');
    disp('Check if Arduino board is connected to computer');
end

%% Rebuild S-functions for local machine
disp('Building Arduino Library blocks');

switch (OperatingSystem)
    case 'windows'
        cd ([ProjDir,'\Arduino\blocks\ADC_IMU']);
        mex Arduino_ADCRead_sfcn.c
        
        cd ([ProjDir,'\Arduino\blocks\Analog']);
        mex sfunar_analogInput.c
        mex sfunar_analogOutput.c
        
        cd ([ProjDir,'\Arduino\blocks\Digital']);
        mex sfunar_digitalInput.c
        mex sfunar_digitalOutput.c
        
        cd ([ProjDir,'\Arduino\blocks\EEPROM']);
        mex Arduino_EEPROMRead_sfcn.c
        
        cd ([ProjDir,'\Arduino\blocks\LCD']);
        mex sfunar_lcdOutput.c
        
        cd ([ProjDir,'\Arduino\blocks\RCChannels']);
        mex Arduino_RCWrite_sfcn.c
        mex Arduino_RCRead_sfcn.c
        
        cd ([ProjDir,'\Arduino\blocks\Serial']);
        mex sfunar_serialConfig.c
        mex sfunar_serialRead.c
        mex sfunar_serialWrite.c
        
        cd ([ProjDir,'\Arduino\blocks/Servo']);
        mex sfunar_servoOutput.c
        
    case 'ubuntu1204'
        cd ([ProjDir,'/blocks/ADC_IMU']);
        mex Arduino_ADCRead_sfcn.c
        
        cd ([ProjDir,'/blocks/Analog']);
        mex sfunar_analogInput.c
        mex sfunar_analogOutput.c
        
        cd ([ProjDir,'/blocks/Digital']);
        mex sfunar_digitalInput.c
        mex sfunar_digitalOutput.c
        
        cd ([ProjDir,'/blocks/EEPROM']);
        mex Arduino_EEPROMRead_sfcn.c
        
        cd ([ProjDir,'/blocks/LCD']);
        mex sfunar_lcdOutput.c
        
        cd ([ProjDir,'/blocks/RCChannels']);
        mex Arduino_RCWrite_sfcn.c
        mex Arduino_RCRead_sfcn.c
        
        cd ([ProjDir,'/blocks/Serial']);
        mex sfunar_serialConfig.c
        mex sfunar_serialRead.c
        mex sfunar_serialWrite.c
        
        cd ([ProjDir,'/blocks/Servo']);
        mex sfunar_servoOutput.c
end

%% Complete setup
cd (ProjDir);
disp('Arduino Setup complete');