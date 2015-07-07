function sl_customization(cm)
% SL_CUSTOMIZATION for Arduino PIL connectivity config

% Copyright 2008-2010 The MathWorks, Inc.

cm.registerTargetInfo(@loc_createSerialConfig);

% local function
function config = loc_createSerialConfig

config = rtw.connectivity.ConfigRegistry;
config.ConfigName = 'Arduino connectivity config using serial';
config.ConfigClass = 'arduino.ConnectivityConfig';

% matching system target file
config.SystemTargetFile = {'arduino.tlc'};

% match any template makefile
config.TemplateMakefile = {};

% match any hardware implementation
config.TargetHWDeviceType = {};
