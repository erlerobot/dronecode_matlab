classdef TimerOldStyle < rtw.connectivity.Timer
% TIMEROLDSTYLE is a Timer subclass to get timing information for the application 
% running on the Arduino target
%
%   TIMER(TARGETAPPLICATIONFRAMEWORK) obtains DATATYPE, SOURCEFILE,
%   HEADERFILE, TIMERFUNCTION and SECONDSPERTICK and passes them
%   as arguments to the TIMER superclass
%
%   See also RTW.CONNECTIVITY.TIMER
%
%   Copyright 2009-2010 The MathWorks, Inc.

    methods

        function this = TimerOldStyle(targetApplicationFramework)
        % Initializes the properties of the timer class
            timerDatatype = 'uint32_T';
            headerFile = fullfile(arduino.Prefs.getArduinoPath,...
                                  'hardware',...
                                  'arduino',...
                                  'cores',...
                                  'arduino',...
                                  'wiring.h');
            
            % The micros() function returns microseconds
            secondsPerTick = 1e-6; 
            timerSourceFile = fullfile(arduino.Prefs.getArduinoPath,...
                                       'hardware',...
                                       'arduino',...
                                       'cores',...
                                       'arduino',...
                                       'wiring.c');
            
            timerFunction = 'micros()';
            
            % call the super class constructor to register the properties
            this@rtw.connectivity.Timer(timerDatatype, timerSourceFile,...
                                        secondsPerTick, timerFunction, headerFile,...
                                        targetApplicationFramework);

        end  
    end
end

