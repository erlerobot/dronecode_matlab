classdef Prefs
%PREFS gives access to Arduino preferences
%
%    This is an undocumented class. Its methods and properties are likely to
%   change without warning from one release to the next.
%
%   Copyright 2009-2011 The MathWorks, Inc.
    
    methods (Static, Access=public)
    
        function setArduinoPath(toolPath) 
            
            if ~exist('toolPath', 'var') || ~ischar(toolPath) 
               nl = sprintf('\n');
               error('RTW:arduino:invalidArduinoPath', ...
                      ['Arduino path must be a string, e.g.' nl ...
                       '   arduino.Prefs.setArduinoPath(''c:\\arduino-0022'')']);
            end
                
            if ~exist(toolPath,'dir')
                error('RTW:arduino:invalidArduinoPath', 'The specified folder (%s) does not exist', toolPath);
            end
                        
            if ~exist(fullfile(toolPath, 'arduino.exe'), 'file')
                error('RTW:arduino:invalidArduinoPath', 'The specified folder (%s) does not contain arduino.exe', toolPath);
            end
            
            % remove trailing backslashes
            toolPath = regexprep(toolPath, '\\+$', '');            
            
            % Alternate form of path to handle spaces
            altPath = RTW.transformPaths(toolPath,'pathType','alternate');            
            
            arduino.Prefs.setPref('ArduinoPath', altPath);           
            
            % The board data is tied to a specific version of Arduino IDE, so if we change
            % the Arduino path (possibly to a different IDE) the existing data may not be valid
            arduino.Prefs.setPref('ArduinoBoard', []);
        end
        
        %%
        function toolPath = getArduinoPath
            toolPath = arduino.Prefs.getPref('ArduinoPath');
            % check validity of path (in case the folder got deleted between
            % after setArduinoPath and before getArduinoPath)
            if ~exist(toolPath,'dir') 
                nl = sprintf('\n');
                error('RTW:arduino:invalidArduinoPath', ...
                      ['Arduino path is unspecified or invalid.' nl ...
                       'Specify a valid path using arduino.Prefs.setArduinoPath, e.g.' nl ...
                       '   arduino.Prefs.setArduinoPath(''c:\\arduino-0022'')']);
            end
        end

        %%
        function setBoard(boardLabel) 
            boardsFile = fullfile(arduino.Prefs.getArduinoPath(), 'hardware', 'arduino', 'boards.txt');
            
            if ~exist(boardsFile, 'file')
                nl = sprintf('\n');
                error('RTW:arduino:invalidArduinoPath', ...
                      ['Unable to find board specification file. Ensure that' nl ...
                       'the path to the Arduino IDE is set correctly, e.g.' nl ...
                       '  arduino.Prefs.setArduinoPath(''c:\arduino-0022'')'] );              
            end
            boards = arduino.Prefs.parseBoardsFile(boardsFile);
            if isempty(boards)
                error('RTW:arduino:invalidBoardSpecification', ...
                      'Unable to read board specification file (%s)', boardsFile);
            end            
            validBoards = fields(boards);  
            
            if ~exist('boardLabel','var')|| ~ischar(boardLabel) || ~any(strcmpi(validBoards, boardLabel))                
                msg = sprintf('Specify one of the following board labels:\n');
                for i=1:numel(validBoards)
                    board = boards.(validBoards{i});
                    paddedLabel = ['''' validBoards{i} ''''];
                    msg = [msg sprintf('  %12s (%s)\n', paddedLabel, board.name)];
                end
                msg = [msg sprintf('\n Example:\n    arduino.Prefs.setBoard(''uno'')')];
                error('RTW:arduino:invalidBoardLabel', msg);                
            end
            
            specifiedBoard = boards.(boardLabel);
            specifiedBoard.label = boardLabel;
            arduino.Prefs.setPref('ArduinoBoard', specifiedBoard);
        end
        
        %%
        function [boardLabel, allData] = getBoard            
            board = arduino.Prefs.getPref('ArduinoBoard');
            if isempty(board)
                nl = sprintf('\n');
                error('RTW:arduino:noBoardSpecification', ...
                      ['Arduino board is not yet specified. ' nl ...
                       'Specify the board using arduino.Prefs.setBoard, e.g.' nl ...
                       '  arduino.Prefs.setBoard(''uno'') ']);
            end
            boardLabel = board.label;
            if nargout == 2
                allData = board;
            end
        end
        
        %%
        function mcu = getMcu
            [~, board] = arduino.Prefs.getBoard;
            mcu = board.build.mcu;
        end                

        function uploadRate = getUploadRate
            [~, board] = arduino.Prefs.getBoard;
            uploadRate = board.upload.speed; % returned as a string
        end
        
        function programmer = getProgrammer
            [~, board] = arduino.Prefs.getBoard;
            programmer = board.upload.protocol;
        end
        
        function cpu_freq = getCpuFrequency()
            [~, board] = arduino.Prefs.getBoard;
            cpu_freq = int32(sscanf(board.build.f_cpu,'%u'));
        end


        function setMcu(~)
            nl = sprintf('\n');
            error('RTW:arduino:deprecated', ...
                  ['This function is deprecated. Use arduino.Prefs.setBoard instead, e.g.' nl ...
                   '  arduino.Prefs.setBoard(''uno'') ']);
        end                

        %%
        function port = getComPort
            port = arduino.Prefs.getPref('ComPort');
            
            if isempty(port)
                nl = sprintf('\n');                
                msg = [
                    'The Arduino serial port must be set and you must have installed' nl ...
                    'the device drivers for your Arduino hardware. ' nl ...
                    ' 1. Install the drivers and connect the Arduino hardware. ' nl ...
                    ' 2. Identify the virtual serial (COM) port. You can do this through' nl ...
                    '    the Windows Device Manager, or by running arduino.Prefs.setComPort' nl ...
                    ' 3. Set the correct COM port using arduino.Prefs.setComPort' ...
                    ];                
                error('RTW:arduino:invalidComPort', msg);
            end
          
            
        end                

        function setComPort(port)            
            if ~exist('port', 'var') || ~ischar(port) || isempty(port)
                nl = sprintf('\n');
                error('RTW:arduino:invalidComPort', ...
                      ['Specify the COM port as a string. E.g.: ' nl ...
                       '   arduino.Prefs.setComPort(''COM8'') ']);
            end
            arduino.Prefs.setPref('ComPort', port);
        end
        
        function ports = searchForComPort(regCmdOutput)
            ports='';
            
            if ispc
                if nargin < 1
                    regCmd=['reg query '...
                        'HKEY_LOCAL_MACHINE\HARDWARE\DEVICEMAP\SERIALCOMM'];
                    [~,regCmdOutput]=system(regCmd);
                end
                
                deviceName='\\Device\\(VCP\d|USBSER\d{3})';
                reg_sz = 'REG_SZ';
                portNum = 'COM\d+';
                expr = [deviceName '\s+' reg_sz '\s+(' portNum ')'];
                allPorts=regexp(regCmdOutput,expr,'tokens');
                if ~isempty(allPorts)
                    ports=cell(1, length(allPorts));
                    for j=1:length(allPorts)
                        ports{j}=allPorts{j}{2};
                    end
                end
            end
        end
        
        
        function timers = getTimersSpecs()
            % This function returns full specification for every available
            % timer on the current target.
            timers = { struct('max_tcnt', int32(256), 'prescalers', int32([1 8 32 64 128 256 1024])), ...
                       struct('max_tcnt', int32(65536), 'prescalers', int32([1 8 64 256 1024])) };
        end

    end
    
    methods(Static,Access=private)
        
        function setPref(prefName, prefValue)
            prefGroup='ArduinoGeneric';
            setpref(prefGroup, prefName, prefValue);
        end
        
        function prefValue = getPref(prefName)
            prefGroup='ArduinoGeneric';
            if ispref(prefGroup,prefName)
                prefValue=getpref(prefGroup, prefName);
            else
                prefValue='';
            end
        end
                
        function boards = parseBoardsFile(filename)
            boards = [];
            fid = fopen(filename,'rt');
            if fid < 0,
                return;
            end
% Changed to fix error encountered with new blocks.txt file for Arduino 1.0  - RAR
%             txt = textscan(fid,'%s','commentstyle','#','delimiter','\n','multipledelimsasone',true);
            txt = textscan(fid,'%s','commentstyle','#','delimiter','\n');
            fclose(fid);
            rawLines = txt{1};
            
            parsedLines = regexp(rawLines,'^([^=]+)=([^$]+)$','tokens');
            evalstr = '';
            for i=1:numel(parsedLines)
                if (~isempty(parsedLines{i}) ) % Added to catch empty cells -  RAR
                    lhs = parsedLines{i}{1}{1};  % can be of the form xx.yy.zz
                    rhs = strrep(parsedLines{i}{1}{2}, '''', ''''''); % escape single quotes
                    evalstr = [evalstr sprintf('boards.%s=''%s'';\n', lhs, rhs)]; %#ok
                end
            end            
            
            eval(evalstr);
        end
        
    end
end

% LocalWords:  USBSER tcnt prescalers
