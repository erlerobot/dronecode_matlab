function arduino_make_rtw_hook(hookMethod, modelName, ~, ~, ~, ~)
% ARDUINO_MAKE_RTW_HOOK

% Copyright 1996-2011 The MathWorks, Inc.


  switch hookMethod
   case 'error'
    % Called if an error occurs anywhere during the build.  If no error occurs
    % during the build, then this hook will not be called.  Valid arguments
    % at this stage are hookMethod and modelName. This enables cleaning up
    % any static or global data used by this hook file.
    disp(['### Build procedure for model: ''' modelName...
          ''' aborted due to an error.']);
      
   case 'entry'
    % Called at start of code generation process (before anything happens.)
    % Valid arguments at this stage are hookMethod, modelName, and buildArgs.
    i_arduino_setup(modelName);
    
   case 'before_tlc'
    % Called just prior to invoking TLC Compiler (actual code generation.)
    % Valid arguments at this stage are hookMethod, modelName, and
    % buildArgs
    
   case 'after_tlc'
    % Called just after to invoking TLC Compiler (actual code generation.)
    % Valid arguments at this stage are hookMethod, modelName, and
    % buildArgs
    
    % This check must be done after the model has been compiled otherwise
    % sample time may not be valid
    i_check_tasking_mode(modelName)


   case 'before_make'
    % Called after code generation is complete, and just prior to kicking
    % off make process (assuming code generation only is not selected.)  All
    % arguments are valid at this stage.
    i_write_arduino_makefiles;

   case 'after_make'
    % Called after make process is complete. All arguments are valid at 
    % this stage.    
    
    % allow a back door for tests to skip download to hardware
    downloadToArduinoHardware = 1;
    if evalin('base','exist(''downloadToArduinoHardware'')')
        downloadToArduinoHardware = evalin('base', 'downloadToArduinoHardware' );
    end
    
    if ~i_isPilSim && ~i_isModelReferenceBuild(modelName) &&...
            downloadToArduinoHardware
        i_download(modelName)
    end
    
   case 'exit'
    % Called at the end of the build process.  All arguments are valid at this
    % stage.
    
    disp(['### Successful completion of build ',...
          'procedure for model: ', modelName]);
      
  end

function i_arduino_setup(modelName)

if ~i_isPilSim
    % Check that the the main function will be generated using the correct 
    % .tlc file
    if bdIsLoaded(modelName) && ~i_isModelReferenceBuild(modelName)        
        requiredSetting = 'arduino_file_process.tlc';
        assert(strcmp(get_param(modelName, 'ERTCustomFileTemplate'), ...
                      requiredSetting),...
               'The model %s must have ERTCustomFileTemplate set to %s.',...
               modelName, requiredSetting);
    end
end


% Check for C_INCLUDE_PATH
if ~isempty(getenv('C_INCLUDE_PATH'))
    error('RTW:arduino:nonEmptyCIncludePath',...
          ['The environment variable C_INCLUDE_PATH is set. '...
           'This may conflict with the gcc for AVR. You should '...
           'clear this environment variable, e.g. by running '...
           'setenv(''C_INCLUDE_PATH'','''') from the MATLAB command '...
           'window.']);
end

disp(['### Starting Arduino build procedure for ', ...
      'model: ',modelName]);
  
  [arduino_path, mcu, f_cpu] = ...
    i_get_arduino_env_settings;


if ~isempty(strfind(pwd,' ')) || ~isempty(strfind(pwd,'&'))
    error('RTW:arduino:pwdHasSpaces',...
          ['The current working folder, %s, contains either a space or ' ...
           'ampersand character. This is '...
           'not supported. You must change the current working folder to '...
           'a path that does not contain either of these characters.'], pwd);
end
  
% Display current settings in build log
disp('###')
disp('### Arduino environment settings:')
disp('###')
fprintf('###     ARDUINO_ROOT:   %s\n',arduino_path)
fprintf('###     MCU:            %s\n',mcu)
fprintf('###     F_CPU:          %s\n',f_cpu)
disp('###')

function i_check_tasking_mode(modelName)

% No support for multi tasking mode
if ~i_isModelReferenceBuild(modelName)  &&  ~i_isPilSim
    solverMode = get_param(modelName,'SolverMode');
    st = get_param(modelName,'SampleTimes');
    if length(st)>1 && ~strcmp(solverMode,'SingleTasking')
        error('RTW:arduino:noMultiTaskingSupport',...
              ['The multi-tasking solver mode is not supported for the real-time '...
               'Arduino target. '...
               'In Simulation > Configuration Parameters > Solver you must select '...
               '"SingleTasking" from the pulldown "Tasking mode for periodic sample '...
               'times".']);
    end
end


function i_write_arduino_makefiles

[arduino_path, mcu, f_cpu] = ...
    i_get_arduino_env_settings;

lCodeGenFolder = Simulink.fileGenControl('getConfig').CodeGenFolder;
buildAreaDstFolder = fullfile(lCodeGenFolder, 'slprj');

% Copy the arduino version of target_tools.mk into the build area
tgtToolsFile = 'target_tools.mk';
target_tools_folder = mfilename('fullpath');
target_tools_folder = fileparts(target_tools_folder);
srcFile = fullfile(target_tools_folder, tgtToolsFile);
dstFile = fullfile(buildAreaDstFolder, tgtToolsFile);
copyfile(srcFile, dstFile, 'f');
% Make sure the file is not read-only
fileattrib(dstFile, '+w');

% gmake needs forward slash as path separator
arduino_path = strrep(arduino_path, '\', '/');

% Write out the makefile
makefileName = fullfile(buildAreaDstFolder, 'arduino_prefs.mk');
fid = fopen(makefileName,'w');
fwrite(fid, sprintf('%s\n\n', '# Arduino build preferences'));
fwrite(fid, sprintf('ARDUINO_ROOT=%s\n', arduino_path));
fwrite(fid, sprintf('MCU=%s\n', mcu));
fwrite(fid, sprintf('F_CPU=%s\n', f_cpu));
fclose(fid);
  
  
function i_download(modelName)
    
    hexFile = fullfile('.',[modelName '.hex']);
    arduino.runAvrDude(hexFile);
    
function isPilSim = i_isPilSim
    s = dbstack;
    isPilSim = false;
    for i=1:length(s)
        if strfind(s(i).name,'build_pil_target')
            isPilSim=true;
            break;
        end
    end
    
function isMdlRefBuild = i_isModelReferenceBuild(modelName)
    mdlRefTargetType = get_param(modelName, 'ModelReferenceTargetType');
    isMdlRefBuild = ~strcmp(mdlRefTargetType, 'NONE');
    

function [arduino_path, mcu, f_cpu] = ...
    i_get_arduino_env_settings

arduino_path = arduino.Prefs.getArduinoPath;
arduino_path = RTW.transformPaths(arduino_path);

mcu = arduino.Prefs.getMcu;
f_cpu = sprintf('%d', arduino.Prefs.getCpuFrequency);
