function blkStruct = slblocks
%% This code was developed as part of graduate coursework at Embry-Riddle Aeronautical University
%% under the guidance of Dr. Pat Anderson and Dr. Hever Moncayo.

%% Author: Robert F. Hartley
%% Assistant Developers: Francois Hugon, Brian DeRosa, and Christopher Carvalho
%% Support: apm2.simulink@gmail.com

%% Oct. 25, 2012
%% Version 1.0



blkStruct.Name = 'ArduPlane 2 Target'; %Display name
blkStruct.OpenFcn = 'ArduPilot2_lib'; %Library name
blkStruct.MaskDisplay = '';
Browser(1).Library = 'ArduPilot2_lib'; %Library name

%   Copyright 2010 The MathWorks, Inc.

Browser(1).Name='ArduPilot 2 Target';%Display name
blkStruct.Browser = Browser;
