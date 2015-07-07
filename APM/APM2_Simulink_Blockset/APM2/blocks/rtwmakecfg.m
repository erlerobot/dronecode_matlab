function makeInfo=rtwmakecfg()
%% This code was developed as part of graduate coursework at Embry-Riddle Aeronautical University
%% under the guidance of Dr. Pat Anderson and Dr. Hever Moncayo.

%% Author: Robert F. Hartley
%% Assistant Developers: Francois Hugon, Brian DeRosa, and Christopher Carvalho
%% Support: apm2.simulink@gmail.com

%% Oct. 25, 2012
%% Version 1.0



%RTWMAKECFG adds include and source directories to the generated makefiles.
%   For details refer to documentation on the rtwmakecfg API.

% Copyright 1994-2011 The MathWorks, Inc.

makeInfo.sources = {'io_wrappers.cpp', 'ADC_wrapper.cpp', 'RC_wrapper.cpp'};


arduino_path = arduino.Prefs.getArduinoPath;
arduino_inc_path = fullfile(arduino_path, 'hardware', 'arduino',...
                            'cores', 'arduino');

% Add the folder where this file resides to the include path
blocks_inc_path = mfilename('fullpath');
blocks_inc_path = fileparts(blocks_inc_path);

makeInfo.includePath = {arduino_inc_path, blocks_inc_path};

