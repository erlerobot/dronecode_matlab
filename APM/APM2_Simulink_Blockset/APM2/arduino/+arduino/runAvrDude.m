function runAvrDude(hexFile)
%RUNAVRDUDE downloads a program into flash using avrdude
    
%   Copyright 2009-2011 The MathWorks, Inc.
    
    persistent lastComPort;
    persistent lastHexContent;
    
    hexContent = fileread(hexFile);
    
    [hexDir,hexFile,hexExt]=fileparts(hexFile);
    hexFile = [hexFile hexExt];
    arduinoPath = arduino.Prefs.getArduinoPath;
    avrPath=fullfile(arduinoPath,'hardware','tools','avr');
    avrPath=strrep(avrPath,'\','/');
    
    avrdude_port = arduino.Prefs.getComPort;
    avrdude_conf = sprintf('%s/etc/avrdude.conf', avrPath);
    avrdude_programmer = arduino.Prefs.getProgrammer;
    avrdude =  sprintf('%s/bin/avrdude', avrPath);
    mcu = arduino.Prefs.getMcu;
    uploadRate = arduino.Prefs.getUploadRate;
    avrdude_flags = sprintf('-V -F -C %s  -p %s -P //./%s -c %s -b %s -U flash:w:%s',...
                avrdude_conf, mcu, avrdude_port, avrdude_programmer, uploadRate, hexFile);
    
    
    cmd = sprintf('%s %s',avrdude, avrdude_flags);
    
    needDownload = ...
        isempty(lastHexContent) || ...
        isempty(lastComPort) || ...
        ~strcmp(hexContent,lastHexContent) || ...
        ~strcmp(avrdude_port, lastComPort);
    
    if needDownload
    
        disp(['Running ' cmd]);
        
        disp(['### If your target board has TX and RX LED you should '...
              'see them flashing ... ']);
        
        origDir = cd(hexDir);
        [s,w] = system(cmd);
        cd(origDir)
        
        disp(w)
        
        if (s~=0)
            error('RTW:arduino:downloadFailed',...
                  ['Download failed. Check that you have specified the correct '...
                   'Arduino board (using arduino.Prefs.setBoard), and the '... 
                   'correct serial port (using arduino.Prefs.setComPort).']);
        end
        
        lastComPort = avrdude_port;
        lastHexContent = hexContent;
        
    else
        disp(['Application is already programmed on the Arduino target: ' ...
            'no flash programming is needed'])
        
        % If the application was already programmed, we reset the target.
        % Opening and closing a serial connection will take the DTR signal
        % low and force a reset of the target.
        
        s=serial(avrdude_port);
        fopen(s);
        fclose(s);
        delete(s);
    end
    
