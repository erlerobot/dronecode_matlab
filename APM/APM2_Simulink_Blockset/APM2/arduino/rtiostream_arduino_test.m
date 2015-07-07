function rtiostream_arduino_test
%RTIOSTREAM_ARDUINO_TEST

% Copyright 2009-2010 The MathWorks, Inc.

% Create a test directory
    testDir=mfilename;
    if isempty(strfind(pwd,testDir))
        if ~exist(testDir,'dir')
            mkdir(testDir)
        end
        origDir = cd(testDir);
        c1 = onCleanup(@()cd(origDir));
    end

    % Copy required files
    srcDir=fullfile(matlabroot,'toolbox','rtw','targets','arduino','arduino');
    copyfile(fullfile(srcDir,'rtiostream_serial.cpp'),'.','f');
    copyfile(fullfile(srcDir,'rtiostream_serial_test.c'),'.','f');
    copyfile(fullfile(srcDir,'rtiostream_serial_test.mk'),'.','f');
    copyfile(fullfile(matlabroot,'rtw','c','src','rtiostream.h'),'.','f');
    
    mcu = arduino.Prefs.getMcu;
    uploadRate = arduino.Prefs.getUploadRate;

    result=evalc(sprintf(['!gmake -f rtiostream_serial_test.mk ' ...
                  'INSTALL_DIR=%s ' ...
                  'MCU=%s ' ...
                  'F_CPU=%s ' ...
                  'PORT=//./%s ' ...
                  'UPLOAD_RATE=%s'],...
                 arduino.Prefs.getArduinoPath,...
                 mcu,...
                 '16000000',...
                 arduino.Prefs.getComPort, ...
                 uploadRate ...
                 ));
    disp(result)
    assert(~isempty(...
        regexp(result, 'avrdude: [0-9]* bytes of flash written','ONCE')),...
           'Log must contain string to confirm data was written to flash')

    s = serial(arduino.Prefs.getComPort);
    fopen(s);

    function serialCleanup(s)
        fclose(s);
        delete(s);
    end

    c2 = onCleanup(@()serialCleanup(s));
    timeout = 10;

    % Wait for the byte transmitted by Arduino that indicates
    % reset is complete and rtIOStreamOpen has been called
    tic
    while (s.BytesAvailable < 1)
        pause(0.1);
        if toc>timeout, break, end
    end
    initCompleteByte=fread(s,1);
    expectedValue=65;
    assert(initCompleteByte==expectedValue,...
           'Target must transmit byte indicating startup is complete');

    maxPacketSize=50;
    for i=1:maxPacketSize
        packetData=(1:i);
        disp(['Sending packet of size ' num2str(i)])
        disp(['Packet data = ' num2str(packetData)])
        fwrite(s,packetData)
        disp(['Waiting for return packet of size ' num2str(i)])
        tic
        while (s.BytesAvailable < i)
            pause(0.1);
            if toc>timeout, break, end
        end
        bytesAvailable=s.BytesAvailable;
        recvdPacket=fread(s,bytesAvailable);
        recvdPacket=recvdPacket';
        disp(['Received packet = '  num2str(recvdPacket)])
        
        expectedPacket=packetData+1;
        if length(expectedPacket)~=length(recvdPacket) || ...
                ~all(expectedPacket==recvdPacket)
            disp(['Expected packet = ' num2str(expectedPacket)])
            result='FAILED';
            break;
        else
            result='Passed';
        end
    end
    
    disp(['Result = ' result])

end
