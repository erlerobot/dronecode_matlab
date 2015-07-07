classdef Launcher < rtw.connectivity.Launcher
%LAUNCHER launches a PIL or real-time application on Arduino
%
%   See also RTW.CONNECTIVITY.LAUNCHER, RTWDEMO_CUSTOM_PIL
    
%   Copyright 2008-2010 The MathWorks, Inc.
   
    methods
        % constructor
        function this = Launcher(componentArgs, builder)
            error(nargchk(2, 2, nargin, 'struct'));
            % call super class constructor
            this@rtw.connectivity.Launcher(componentArgs, builder);
        end
        
        % destructor
        function delete(this) %#ok
        end
        
        % Start the application
        function startApplication(this)
            % get name of the executable file
                
            hexFile = this.getBuilder.getApplicationExecutable;
            
            arduino.runAvrDude(hexFile);
            
            disp('### Starting the PIL simulation')
        end
        
        % Stop the application
        function stopApplication(~)

            disp('### Stopping PIL simulation')
        
        end
    end
end
