classdef Timer < rtw.connectivity.Timer
% TIMER is a Timer subclass to get timing information for the application 
% running on the Arduino target
%
%   See also RTW.CONNECTIVITY.TIMER
%
%   Copyright 2009-2010 The MathWorks, Inc.

    methods

        function this = Timer
            
            % Configure data type returned by timer reads
            this.setTimerDataType('uint32');

            % The micros() function returns microseconds
            ticksPerSecond = 1e6; 
            this.setTicksPerSecond(ticksPerSecond);

            % The timer counts upwards
            this.setCountDirection('up');

            % Configure source files required to access the timer
            headerFile = fullfile(arduino.Prefs.getArduinoPath,...
                                  'hardware',...
                                  'arduino',...
                                  'cores',...
                                  'arduino',...
                                  'wiring.h');
            
            timerSourceFile = fullfile(arduino.Prefs.getArduinoPath,...
                                       'hardware',...
                                       'arduino',...
                                       'cores',...
                                       'arduino',...
                                       'wiring.c');
            
            
            this.setSourceFile(timerSourceFile);
            this.setHeaderFile(headerFile);

            % Configure the expression used to read the timer
            readTimerExpression = 'micros()';
            this.setReadTimerExpression(readTimerExpression);
            
        end  
    end
end

