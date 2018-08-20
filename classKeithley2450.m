classdef classKeithley2450 < handle
% create Keithley2450 object
% This function creates a device object for a Keithley 2450 (TSP).

% INPUT:
%   connectionType: string containing type of connection (serial or gpib)
%   port: cell array of available serial ports (strings, i.e., 'COM[0-255]', '/dev/ttyS[0-255]')
%
% OUTPUT:
%   deviceObject: handle to device object
%
% METHODS:
%   deviceObject = classKeithley2400
%	reset
%	connect
%	disconnect
%	refresh
%	connectionHandle = getConnection
%	setConnectionType
%	type = getConnectionType
%	setPort
%	[port,portSet] = getPort
%	[connectionStatus, ID] = getConnectionStatus
%	err = setOutputState
%	outputState = getOutputState
%	err = abort
%	err = abortAll
%	err = goIdle
%	err = setPointV
%	err = updatePointV
%	dataPointV = measurePointV
%	err = setSweepV
%	err = updateSweepV
% 	dataSweepV = measureSweepV
% 	dataSweepVPP = measureSweepVPP
%	dataSweepCycle = measureCycle
% 	dataSteadyStateMPP = measureSteadyState
%	dataSteadyStateJSC = measureSteadyState
%	dataSteadyStateVOC = measureSteadyState
%	err = setTimeScan
%	err = updateTimeScan
%	dataTimeScan = measureTimeScan
% 	dataTimePointV = measureTimePointV
%	dataTimeSweepV = measureTimeSweepV
%
% Tested: Matlab 2015b, Win10, NI GPIB-USB-HS+ Controller, Keithley KUSB-488B Controller, Keithley 2400, Keithley2410, Keithley2401
% Author: Eugen Zimmermann, Konstanz, (C) 2016 eugen.zimmermann@uni-konstanz.de
% Last Modified on 2018-03-23
    properties (Constant)
        Type = 'device';
        functionName = 'classKeithley2450';
        
        %# set default settings
        defaultID = 'Keithley2450';
        minV = 0;
        maxV = 1;
        intRate = 1;
        compLevel = 2;
        spacing = 'lin';
    end

    properties (Access = private)
        id
        port
        portSet
        connected
        connectionType
        connectionHandle
        abortMeasurement
        abortAllMeasurements
        outputState
        defaultSettings
    end

    methods        
		function device = classKeithley2450(varargin)
            %# get name of function
            ST = dbstack;
            functionNameTemp = ST.name;
        
            input = inputParser;
                addParameter(input,'connectionType','gpib',validationFcn('connectionType',functionNameTemp))
                addParameter(input,'port','none',validationFcn('generalPort',functionNameTemp));
            parse(input,varargin{:});
            
            %# set device id to ''
            device.id = '';
            
            %# set device status to "not connected" and "output deactivated"
            device.connected = 0;
            device.outputState = 0;
            
            %# set abort triggers to clear = 0
            device.abortMeasurement = 0;
            device.abortAllMeasurements = 0;
            
            %# set port if available
            device.connectionType = input.Results.connectionType;
            if strcmpi(input.Results.port,'none')
                switch device.connectionType
                    case 'gpib'
                        device.portSet = 1;
                        device.port = 24;
                    otherwise
                        device.portSet = 0;
                        device.port = [];
                end
            else
                %# if this line throws an error, than check your port parameter
                check_port = validationFcn([device.connectionType,'Port'],functionNameTemp);
                    check_port(input.Results.port)
                
                device.portSet = 1;
                device.port = input.Results.port;
            end
        end
        
        function reset(device,varargin)
            if device.getConnectionStatus()
                device.setOutputState(0);
            end
            device.disconnect();
        end
        
        function connectionHandle = getConnection(device)
            connectionHandle = device.connectionHandle;
        end
        
        function [status,id] = getConnectionStatus(device)
            status = device.connected;
            id = device.id;
        end
        
        function setConnectionType(device,connectionType)
            input = inputParser;
                addRequired(input,'device')
                addRequired(input,'connectionType',validationFcn('connectionType',device.functionName))
            parse(input,device,connectionType);
            
            if ~strcmpi(input.Results.connectionType,device.connectionType)
                device.connectionType = input.Results.connectionType;
                device.portSet = 0;
            end
        end
        
        function type = getConnectionType(device)
            type = device.connectionType;
        end

        function setPort(device,port)
            input = inputParser;
                addRequired(input,'device')
                addRequired(input,'port',validationFcn([lower(device.connectionType),'Port'],device.functionName))
            parse(input,device,port);
            
            device.port = input.Results.port;
            device.portSet = 1;
        end
        
        function [port,portSet] = getPort(device)
            port = device.port;
            portSet = device.portSet;
        end
        
        function manConnect(device,state)
            input = inputParser;
                addRequired(input,'device')
                addRequired(input,'state',validationFcn('boolean',device.functionName))
            parse(input,device,state);
            
            if state
                device.connect();
            else
                device.disconnect();
            end
        end
        
        function refresh(device)
            device.manConnect(0);
            device.manConnect(1);
        end
        
        function connect(device)
            if device.portSet
                try
					device_temp = device.initialize(device.connectionType, device.port);

                    %# check if program is connected to correct device
                    id_temp = ['Keithley',query(device_temp,'print(localnode.model)')];
                    device.connected = strfind(lower(id_temp),lower(device.defaultID));
                    if device.connected
                        %# assign connection handle to device object
                        device.connectionHandle = device_temp;
                        device.id = id_temp;
                        
                        %# Control auto zero (OFF = disabled; ON = enabled; ONCE = force immediate update of auto zero.)
                        %# Helpful for drifting zero as, i.e., when the device is heating up
                        device.setAutoZero(1);
                        
                        %# reset time at startup
                        device.resetTimer();

                        %# Enable/disable timestamp reset when exiting idle.
%                         fprintf(device.connectionHandle,':SYST:TIME:RES:AUTO OFF');

                        %# Select timestamp format (ABSolute or DELTa).
%                         fprintf(device.connectionHandle,':TRAC:TST:FORM ABS');

                        %# Specify buffer control mode (NEVER or NEXT).
%                         fprintf(device.connectionHandle,':TRAC:FEED:CONT NEVER');    
                        
                        disp(['connected to ',device.defaultID])
                    else
                        errordlg(sprintf(getErrorMessage('deviceNotFound'),device.defaultID,device.port,id_temp))
                        fclose(device_temp);
                        delete(device_temp);
                        return
                    end
                catch err
                    errordlg(err.message)
                    try
                        fclose(device_temp);
                        delete(device_temp);
                    catch err2
                        disp(err2.message)
                    end
                    device.connected = 0;
                    device.connectionHandle = [];
                    return
                end
            else
                helpdlg(['Please define a ',device.connectionType,'-port first.'],'Port not set')
                return
            end
        end
        
        function disconnect(device)
            try
                if isa(device.connectionHandle,'serial') || isa(device.connectionHandle,'gpib')
                    fclose(device.connectionHandle);
                    device.connectionHandle = [];
                    disp(['disconnected from ',device.defaultID])
                end
                device.connected = 0;
                device.id = '';
            catch error
                errordlg(error.message)
            end
        end
        
		function getEventlogMessages(device)
            ec = device.getEventlogCount();
            for n1=1:ec
                disp(device.readEventlog());
            end
            
        end								
        function state = getOutputState(device)
            state = device.outputState;
        end
        
        function err = setOutputState(device,varargin)           
            input = inputParser;
                addRequired(input,'device')
                addOptional(input,'state',0,validationFcn('boolean',device.functionName))
            parse(input,device,varargin{:});
            
            %# check if device is connected, otherwise abort
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            device.clearDevice();

            %# deactivate output
            err = device.toggleOutput(input.Results.state);
            if ~err
                device.outputState = input.Results.state;
                device.abortMeasurement = 0;
                device.abortAllMeasurements = 0;
            end
        end
        
        %# set trigger in order to abort/skip current measurement
        function err = abort(device)
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            device.abortMeasurement = 1;
            device.clearDevice();
            err = device.setOutputState(0);
        end
        
        function err = abortAll(device)
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            device.abortAllMeasurements = 1;
            device.clearDevice();
            err = device.setOutputState(0);
        end
        
        function err = goIdle(device)
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            device.abortMeasurement = 0;
            device.abortAllMeasurements = 0;
%             device.clearDevice();
        end
        
        function playStarWars(device)
            if ~device.connected
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            
            device.startStarWars();
        end
        
        %# prepare settings for single point measurement
        function err = setPointV(device,sourceV,delay,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'sourceV',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',device.intRate,validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'complianceLevel',device.compLevel,validationFcn('keithleySetV',device.functionName));
			addParameter(input,'points',1,validationFcn('keithleyDuration',device.functionName));
            parse(input,device,sourceV,delay,varargin{:})

            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end

            %# set voltage as source
            err = device.setSourceMode('V');
            if err
                errordlg(sprintf(getErrorMessage('keithleySetSourceV'),device.functionName))
                return
            end

            %# set point parameter as source voltage and delay
            err = device.updatePointV(input.Results.sourceV,input.Results.delay,'points',input.Results.points);
            if err
                return
            end
            
            %# set sense function (device, mode [I,V,R], int_rate, prot_lvl, range_auto, range)
            err = device.setSense('I', 'integrationRate',input.Results.integrationRate,'complianceLevel',input.Results.complianceLevel);
            if err
                errordlg(sprintf(getErrorMessage('keithleySetSourceV'),device.functionName))
                return
            end
            
%             fprintf(device.connectionHandle,':FORM:ELEM:SENS VOLT,CURR,TIME'); % Specify data elements (VOLTage, CURRent,RESistance, TIME, and STATus).

            if ~device.outputState
                %# activate output if not already active
                err = device.toggleOutput(1);
                if err
                    return
                end
                device.outputState = 1;
            end
        end

        function err = updatePointV(device,sourceV,delay,varargin)            
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'sourceV',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addParameter(input,'mode','V',validationFcn('keithleyMode',device.functionName));
			addParameter(input,'points',1,validationFcn('keithleyDuration',device.functionName));
            parse(input,device,sourceV,delay,varargin{:})

            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end

            %# set sweep mode to single point
            err = device.setSingle(input.Results.mode, input.Results.sourceV,'points',input.Results.points);
            if err
                return
            end

            %# set delay between each measurement point
            err = device.setDelay('manual', input.Results.delay);
            if err
                return
            end
        end

        %# measure function single point
        function [outputData,err] = measurePointV(device)
            input = inputParser;
            addRequired(input,'device')
            parse(input,device)
            
            %# initialize default values
            outputData = struct();

            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            elseif ~device.outputState
                %# activate output if not already active
                err = device.toggleOutput(1);
                if err
                    return
                end
                device.outputState = 1;
            end
            
            %# read measured values
            device.initTriggerModel()
            device.waitComplete()
            
            %# extract double from scanned string
            numStr = device.readBuffer();
            nums = str2double(strsplit(numStr(1:end-1),','));

            %# assign measured values to variables
            outputData.voltage=nums(1);
            outputData.current=nums(2);
            outputData.time = nums(3);
            
            device.getEventlogMessages();
            err = 0;
        end
        
        function err = setSweepV(device,start,stop,step,delay,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'start',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'stop',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'step',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',device.intRate,validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'spacing',device.spacing,validationFcn('keithleySpacing',device.functionName));
            parse(input,device,start,stop,step,delay,varargin{:})

            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end

            %# set voltage as source
            err = device.setSourceMode('V');
            if err
                errordlg(sprintf(getErrorMessage('keithleySetSourceV'),device.functionName))
                return
            end

            err = device.updateSweepV(input.Results.start,input.Results.stop,input.Results.step,input.Results.delay,input.Results.integrationRate,'spacing',input.Results.spacing);
            if err
                return
            end

            if ~device.outputState
                %# activate output if not already active
                err = device.toggleOutput(1);
                if err
                    return
                end
                device.outputState = 1;
            end
        end

        function err = updateSweepV(device,start,stop,step,delay,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'start',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'stop',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'step',validationFcn('keithleySetStepV',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',device.intRate,validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'complianceLevel',device.compLevel,validationFcn('keithleySetV',device.functionName));
            addParameter(input,'spacing',device.spacing,validationFcn('keithleySpacing',device.functionName));
            parse(input,device,start,stop,step,delay,varargin{:})

            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            
            %# set sweep with given values for start, stop, step, spacing ('LIN', and 'LOG')
            err = device.setSweep('V', input.Results.start, input.Results.stop, input.Results.step,'spacing',input.Results.spacing);
            if err
                return
            end

            %# set delay between each measurement point
            err = device.setDelay('manual', input.Results.delay);
            if err
                return
            end

            %# set sense function (device, mode [I,V,R], int_rate, prot_lvl, range_auto, range)
            err = device.setSense('I', 'integrationRate',input.Results.integrationRate,'complianceLevel',input.Results.complianceLevel);
            if err
                return
            end
        end

        %# measure function sweep
        function [outputData,err] = measureSweepV(device,varargin)
            input = inputParser;
            addRequired(input,'device')
            addParameter(input,'plotHandle',0,@(x) (isnumeric(x) && x==0) || isgraphics(x,'axes') || (isstruct(x) && isfield(x,'update')));
            parse(input,device,varargin{:});
            
            ax = input.Results.plotHandle;
            
            %# initialize default values
            outputData = struct();
            outputData.voltage = 0;
            outputData.current = 0;
            outputData.time    = 0;
            if device.abortMeasurement
                err = 88;
                return
            elseif device.abortAllMeasurements
                err = 99;
                return
            end

            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            elseif ~device.outputState
                %# activate output if not already active
                err = device.toggleOutput(1);
                if err
                    return
                end
                device.outputState = 1;
            end

            %# preallocate space for results (Keithley stores internally up to 2500 points)
            voltage = zeros(2500,1);
            current = zeros(2500,1);
            time = zeros(2500,1);

            %# read measured values
            device.initTriggerModel();
            
            n1 = 1;
            %# extract measured values from string while measuring and update plot
            while ~device.abortMeasurement && ~device.abortAllMeasurements && n1<=2500
                try
                    %# read 3 values á 13 digits + comma
%                     tmp = fscanf(device.connectionHandle,'%c',42);
                    tmp = device.readBuffer('startIndex',n1,'endIndex',n1);
                    splitted = strsplit(tmp,',');
                catch eee
                    disp(eee.message);
                    break;
                end

                %# stop reading if end of transmission is reached
                if isempty(tmp) || strcmp(splitted{1},'9.910000e+37')
                    break;
                else
                    nums = str2double(splitted(1:end));
                    voltage(n1) = nums(1);
                    current(n1) = nums(2);
                    time(n1)    = nums(3);
                end

                if isgraphics(ax,'axes')
                    plot(ax,voltage(1:n1),current(1:n1))
                    xlabel(ax,'Voltage (V)')
                    ylabel(ax,'Current (A)')
                elseif isstruct(ax) && isfield(ax,'update')
                    ax.update(voltage(1:n1),current(1:n1),'xlabel','Voltage (V)','ylabel','Current (A)')
                end
                drawnow
                n1 = n1+1;
            end

            outputData.voltage = voltage(1:max(n1-1,1));
            outputData.current = current(1:max(n1-1,1));
            outputData.time    = time(1:max(n1-1,1));
            
            device.getEventlogMessages();

            %# check if scan was aborted by user and adjust status
            if device.abortMeasurement
                device.setOutputState(0);
                device.abortMeasurement = 0;
                err = 88;
                return
            elseif device.abortAllMeasurements
                device.setOutputState(0);
                device.abortAllMeasurements = 0;
                err = 99;
                return
            else
                device.goIdle();
                err = 0;
                return
            end
        end
        
        %# generate prepuls
        function err = prebias(device,prepulsV,duration,delay)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'prepulsV',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'duration',validationFcn('keithleyDuration',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            parse(input,device,prepulsV,duration,delay)

            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            
            err = device.setPointV(input.Results.prepulsV,input.Results.delay);
            if err
                return
            end
            
            pause(input.Results.duration);
        end
        
        %# measureSweepV with single prepuls
        function [outputData,err] = measureSweepVPP(device,start,stop,step,delay,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'start',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'stop',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'step',validationFcn('keithleySetStepV',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',device.intRate,validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'spacing',device.spacing,validationFcn('keithleySpacing',device.functionName));
            addParameter(input,'plotHandle',0,@(x) (isnumeric(x) && x==0) || isgraphics(x,'axes') || (isstruct(x) && isfield(x,'update')));
            addParameter(input,'prepulsV',0,validationFcn('keithleySetV',device.functionName));
            addParameter(input,'prepulsDuration',0,validationFcn('keithleyDelay',device.functionName));
            addParameter(input,'prepulsDelay',0,validationFcn('keithleyDelay',device.functionName));
            parse(input,device,start,stop,step,delay,varargin{:})
            
            outputData = struct();

            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            
            err = device.prebias(input.Results.prepulsV,input.Results.prepulsDuration,input.Results.prepulsDelay);
            if err
                return
            end
            
            err = device.setSweepV(input.Results.start,input.Results.stop,input.Results.step,input.Results.delay,input.Results.integrationRate);
            if err
                return
            end
            
            [outputSweep,err] = device.measureSweepV('plotHandle',input.Results.plotHandle);
            device.setOutputState(0);
            if err
                return
            end
            
            outputData = outputSweep;
        end
        
        function [outputData,err] = measureCycle(device,vmpp,voc,step,delay,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'vmpp',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'voc',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'step',validationFcn('keithleySetStepV',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',device.intRate,validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'complianceLevel',device.compLevel,validationFcn('keithleySetV',device.functionName));
            addParameter(input,'plotHandle',0,@(x) (isnumeric(x) && x==0) || isgraphics(x,'axes') || (isstruct(x) && isfield(x,'update')));
            parse(input,device,vmpp,voc,step,delay,varargin{:})
            
            ax = input.Results.plotHandle;

            outputData = struct();
            outputData.voltage = 0;
            outputData.current = 0;
            outputData.time    = 0;
            if device.abortMeasurement
                err = 88;
                return
            elseif device.abortAllMeasurements
                err = 99;
                return
            end
            
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end

            vmpp = input.Results.vmpp;
            voc = input.Results.voc;
            step = input.Results.step;

            %# MPP->JSC->VOC->JSC
            volt = [vmpp:-step:0, 0:step:voc,voc:-step:0];

            %# set voltage as source
            err = device.setSourceMode('V');
            if err
                errordlg(sprintf(getErrorMessage('keithleySetSourceV'),device.functionName))
                return
            end

            %# set list as sweep
            err = device.setList('V', volt);
            if err
                return
            end
            
            %# set delay between each measurement point
            err = device.setDelay('manual', input.Results.delay);
            if err
                return
            end

            %# set sense function (device, mode [I,V,R], int_rate, prot_lvl, range_auto, range)
            err = device.setSense('I', 'integrationRate',input.Results.integrationRate,'complianceLevel',input.Results.complianceLevel);
            if err
                log.update(['Could not set integrationRate ',num2str(input.Results.integrationRate),' in cycle!'])
                return
            end
            
            if ~device.outputState
                %# activate output if not already active
                err = device.toggleOutput(1);
                if err
                    return
                end
                device.outputState = 1;
            end
            
            %# preallocate space for results (Keithley stores internally up to 2500 points)
            voltage = zeros(2500,1);
            current = zeros(2500,1);
            time = zeros(2500,1);

            %# read measured values
            device.initTriggerModel();
            
            n1 = 1;
            %# extract measured values from string while measuring and update plot
            while ~device.abortMeasurement && ~device.abortAllMeasurements && n1<=2500
                try
                    %# read 3 values á 13 digits + comma
%                     tmp = fscanf(device.connectionHandle,'%c',42);
                    tmp = device.readBuffer('startIndex',n1,'endIndex',n1);
                    splitted = strsplit(tmp,',');
                catch eee
                    disp(eee.message);
                    break;
                end

                %# stop reading if end of transmission is reached
                if isempty(tmp) || strcmp(splitted{1},'9.910000e+37')
                    break;
                else
                    nums = str2double(splitted(1:end));
                    voltage(n1) = nums(1);
                    current(n1) = nums(2);
                    time(n1)    = nums(3);
                end

                if isgraphics(ax,'axes')
                    plot(ax,voltage(1:n1),current(1:n1))
                    xlabel(ax,'Voltage (V)')
                    ylabel(ax,'Current (A)')
                elseif isstruct(ax) && isfield(ax,'update')
                    ax.update(voltage(1:n1),current(1:n1),'xlabel','Voltage (V)','ylabel','Current (A)')
                end
                drawnow
                n1 = n1+1;
            end

            device.setOutputState(0);
            
            outputData.voltage = voltage(1:max(n1-1,1));
            outputData.current = current(1:max(n1-1,1));
            outputData.time    = time(1:max(n1-1,1));
            
            device.getEventlogMessages();

            %# check if scan was aborted by user and adjust status
            if device.abortMeasurement
                device.setOutputState(0);
                device.abortMeasurement = 0;
                err = 88;
                return
            elseif device.abortAllMeasurements
                device.setOutputState(0);
                device.abortAllMeasurements = 0;
                err = 99;
                return
            else
                device.goIdle();
                err = 0;
                return
            end
        end
        
        function err = setDischarge(device,varargin)
            err = 404;
            disp('Not implemented yet')
            return
            
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'sourceV',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',device.intRate,validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'mode','V',validationFcn('keithleyMode',device.functionName));
            addParameter(input,'outputMode','HIMP',validationFcn('keithleyOutputMode',device.functionName))
            addParameter(input,'wireMode',1,validationFcn('boolean',device.functionName))
            parse(input,device,sourceV,delay,varargin{:})
            
            mode = input.Results.mode;
            
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            
            %# clear Trigger events
            fprintf(device.connectionHandle,':TRIGger:CLEar');
            
            %# set 4-wire mode
            fprintf(device,['SYST:RSEN ', con_a_b(input.Results.wireMode,'ON','OFF')]);
            
            %# set output off-state mode
            fprintf(device.connectionHandle ,[':OUTP:SMOD ',input.Results.outputMode]);

            %# set voltage as source
            err = device.setSourceMode(mode);
            if err
                errordlg(sprintf(getErrorMessage('keithleySetSourceV'),device.functionName))
                return
            end
            
            %# set fixed voltage source range
            fprintf(device.connectionHandle ,':SOUR:VOLT:MODE FIXED');
            
            err = device.updateTimeScan(input.Results.sourceV,input.Results.delay,input.Results.integrationRate,'resetTime',1,'mode',mode);
            if err
                return
            end
          
            %# set sense functions
%             fprintf(device.connectionHandle,':FORM:ELEM:SENS VOLT,CURR,TIME');
            fprintf(device.connectionHandle,':FORM:ELEM:SENS CURR,TIME');
            fprintf(device.connectionHandle,':SENS:CURR:RANG:AUTO ON');
            fprintf(device.connectionHandle,':SENS:CURR:PROT 100E-3');
            
            %# Specify trigger count (1 to 2500).
            fprintf(device.connectionHandle,':TRIG:COUN 2500');
            
            device.toggleAutoclear('OFF');
        end
        
        function [outputData,err] = measureDischarge(device,duration,varargin)
            outputData = struct();
            err = 404;
            disp('Not implemented yet')
            return
            
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'duration',validationFcn('keithleyDuration',device.functionName));
            addParameter(input,'hold',0,validationFcn('boolean',device.functionName));
            addParameter(input,'plotHandle',0,@(x) (isnumeric(x) && x==0) || isgraphics(x,'axes') || (isstruct(x) && isfield(x,'update')));
            addParameter(input,'temperatureController',[]);
            parse(input,device,duration,varargin{:})
            
            modeYLabel = 'Current (A)';
            ax = input.Results.plotHandle;
            tC = input.Results.temperatureController;
				measureTemperature = ~isempty(tC) && isfield(tC,'startTemp') && isfield(tC,'endTemp') && isfield(tC,'device') && tC.device.getConnectionStatus();
            
            outputData = struct();
            outputData.current = 0;
            outputData.time    = 0;
            if device.abortMeasurement
                err = 88;
                return
            elseif device.abortAllMeasurements
                err = 99;
                return
            end
            
            %# preallocate memory for results
            endReached = 0;
            current = zeros(2500,1);
            time = zeros(2500,1);
            if measureTemperature
                temperature = zeros(2500,1);
                tempFactor = tC.startTemp>tC.endTemp;
            end
            
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            elseif ~device.outputState
                %# activate output if not already active
                err = device.toggleOutput(1);
                if err
                    return
                end
                device.outputState = 1;
            end
            
            %# clear Trigger events
            fprintf(device.connectionHandle,':TRIGger:CLEar');
            fprintf(device.connectionHandle,':TRIG:COUN 2500');
            
            t0 = query(device.connectionHandle,':SYST:TIME?');
            if isempty(t0)
                fprintf(device.connectionHandle,':ABORt');
                device.setOutputState(0);
                err = -2;
                return
            else
                t0 = str2double(t0);
            end
            
            fprintf(device.connectionHandle,':READ?');
            tmp = '';
            a = 0;
            divisor = 2000;
            divisorStep = divisor;
            %# extract measured values from string while measuring and update plot
            while ~device.abortMeasurement && ~device.abortAllMeasurements && ~endReached && a<duration
                if ~device.getOutputState()
                    outputData.current = 0;
                    outputData.time    = 0;
                    err = 88;
                    return;
                end
                try
                    tmp = [tmp,fscanf(device.connectionHandle,'%c',56)];
                    splitted = strsplit(tmp,',');
                catch eee
                    disp(eee.message);
%                     device.abortAllMeasurements = 1;
%                     break;
                end
                
                %# stop reading if end of transmission is reached
                if isempty(tmp)
                    break;
                elseif isspace(tmp(end))
                    nums = str2double(splitted(1:end));
                    endReached = 1;
                else
                    nums = str2double(splitted(1:end-1));
                end

                current = nums(1:2:end-1);
                time = nums(2:2:end);
                a = time(end)-time(1);
                if measureTemperature
                    b = length(nonzeros(time));
                    c = length(nonzeros(temperature));
                    outputTemp = tC.device.getTemp();
                    temperature(c+1:b) = repmat(outputTemp.temperature,size(c+1:b));
                    if abs((time(end)-time(1))-tC.delay)<1 && tC.rampActive
                        tC.device.startRamp(tC.rate);
                        tC.device.setTemp(tC.endTemp);
                        disp('ramp started')
                        pause(2)
                    end
                    if con_a_b(tempFactor<0,outputTemp.temperature<tC.endTemp,outputTemp.temperature>tC.endTemp) && (duration-a)>tC.delay
                        duration = a+tC.delay;
                    end
                end
                
                if (length(current)/divisor)>1
                    length(current)
                    divisor = divisor+divisorStep;
                    fprintf(device.connectionHandle,':ABORt');
                    fprintf(device.connectionHandle,':TRIG:COUN 2500');
                    fprintf(device.connectionHandle,':READ?');
                end
                
                modeYData = current(1:ceil(length(current)/250):end);
                if isgraphics(ax,'axes')
                    if input.Results.hold
                        hold on
                    else
                        hold off
                    end
                    
                    plot(ax,time(1:ceil(length(time)/250):end),modeYData,'Color',ax.ColorOrder(1,:))
                    xlabel(ax,'Time (s)')
                    ylabel(ax,modeYLabel)
                    drawnow
                elseif isstruct(ax) && isfield(ax,'update')
                    ax.update(time(1:ceil(length(time)/250):end),modeYData,'xlabel','Time (s)','ylabel',modeYLabel,'hold',input.Results.hold)
                end
            end
            
            outputData.current = current';
            outputData.time = time';
            outputData.t0 = t0';
            if measureTemperature
                outputData.temperature = temperature(1:length(time));
            end
            
            %# check if scan was aborted by user and adjust status
            if device.abortMeasurement
                device.abortMeasurement = 0;
                device.setOutputState(0);
                err = 88;
                return
            elseif device.abortAllMeasurements
                device.abortAllMeasurements = 0;
                device.setOutputState(0);
                err = 99;
                return
            else
                device.goIdle();
                device.setOutputState(0);
                err = 0;
                return
            end
        end
        
        function err = setTimeScan(device,sourceV,delay,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'sourceV',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',device.intRate,validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'mode','V',validationFcn('keithleyMode',device.functionName));
            addParameter(input,'points',1E5,validationFcn('keithleyDuration',device.functionName))
%             addParameter(input,'outputMode','NORM',validationFcn('keithleyOutputMode',device.functionName))
            parse(input,device,sourceV,delay,varargin{:})
            
            mode = input.Results.mode;
            
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            
            %# set voltage as source
            err = device.setSourceMode(mode);
            if err
                errordlg(sprintf(getErrorMessage('keithleySetSourceV'),device.functionName))
                return
            end
            
            err = device.updateTimeScan(input.Results.sourceV,input.Results.delay,input.Results.integrationRate,'resetTime',1,'mode',mode,'points',input.Results.points);
            if err
                return
            end
                        
            if ~device.outputState
                %# activate output if not already active
                err = device.toggleOutput(1);
                if err
                    return
                end
                device.outputState = 1;
            end
        end
        
        function err = updateTimeScan(device,sourceV,delay,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'sourceV',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',device.intRate,validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'complianceLevel',device.compLevel,validationFcn('keithleySetV',device.functionName));
            addParameter(input,'resetTime',0,validationFcn('boolean',device.functionName));
            addParameter(input,'mode','V',validationFcn('keithleyMode',device.functionName));
            addParameter(input,'points',1,validationFcn('keithleyDuration',device.functionName))
            parse(input,device,sourceV,delay,varargin{:})
            
            mode = input.Results.mode;
                modeSense = con_a_b(strcmp(mode,'V'),'I','V');
            
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            
            err = device.updatePointV(input.Results.sourceV,input.Results.delay,'mode',mode,'points',input.Results.points);
            if err
                return
            end
            
            err = device.setSense(modeSense, 'integrationRate',input.Results.integrationRate,'complianceLevel',input.Results.complianceLevel);
            if err
                return
            end
            
            disp('#ToDo implement switch between absolute and relative timestamp')
%             if input.Results.resetTime
%                 fprintf(device.connectionHandle,':SYST:TIME:RES');
%             end
        end
        
        function [outputData,err] = measureTimeScan(device,duration,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'duration',validationFcn('keithleyDuration',device.functionName));
            addParameter(input,'hold',0,validationFcn('boolean',device.functionName));
            addParameter(input,'plotHandle',0,@(x) (isnumeric(x) && x==0) || isgraphics(x,'axes') || (isstruct(x) && isfield(x,'update')));
            addParameter(input,'temperatureController',[]);
            addParameter(input,'mode','V',validationFcn('keithleyMode',device.functionName));
            parse(input,device,duration,varargin{:})
            
            mode = input.Results.mode;
                modeYLabel = con_a_b(strcmp(mode,'V'),'Current (A)','Voltage (V)');
            
            ax = input.Results.plotHandle;
            tC = input.Results.temperatureController;
				measureTemperature = ~isempty(tC) && isfield(tC,'startTemp') && isfield(tC,'endTemp') && isfield(tC,'device') && tC.device.getConnectionStatus();
            
            outputData = struct();
            outputData.voltage = 0;
            outputData.current = 0;
            outputData.time    = 0;
            if device.abortMeasurement
                err = 88;
                return
            elseif device.abortAllMeasurements
                err = 99;
                return
            end
            
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            elseif ~device.outputState
                %# activate output if not already active
                err = device.toggleOutput(1);
                if err
                    return
                end
                device.outputState = 1;
            end
            
            %# preallocate memory for results
            current = zeros(2500,1);
            voltage = zeros(2500,1);
            time = zeros(2500,1);
            if measureTemperature
                temperature = zeros(2500,1);
                tempFactor = tC.startTemp>tC.endTemp;
            end
            
            disp('#ToDo implement switch between absolute and relative timestamp')
            t0 = 0;
%             t0 = query(device.connectionHandle,':SYST:TIME?');
%             if isempty(t0)
%                 fprintf(device.connectionHandle,':ABORt');
%                 device.setOutputState(0);
%                 err = -2;
%                 return
%             else
%                 t0 = str2double(t0);
%             end
            
            %# read measured values
            device.initTriggerModel();
            
            a = 0;
            n1 = 1;
            %# extract measured values from string while measuring and update plot
            while ~device.abortMeasurement && ~device.abortAllMeasurements && a<duration
                if ~device.getOutputState()
                    outputData.voltage = 0;
                    outputData.current = 0;
                    outputData.time    = 0;
                    err = 88;
                    return;
                end
                try
                    tmp = device.readBuffer('startIndex',n1,'endIndex',n1);
                    splitted = strsplit(tmp,',');
                catch eee
                    disp(eee.message);
                    break;
                end
                
                %# stop reading if end of transmission is reached
                if isempty(tmp) || strcmp(splitted{1},'9.910000e+37')
                    break;
                else
                    nums = str2double(splitted(1:end));
                    voltage(n1) = nums(1);
                    current(n1) = nums(2);
                    time(n1)    = nums(3);
                end

                a = time(n1)-time(1);
                if measureTemperature
                    disp('#ToDo check if nonzeros(time) is correct')
                    b = length(nonzeros(time));
                    c = length(nonzeros(temperature));
                    outputTemp = tC.device.getTemp();
                    temperature(c+1:b) = repmat(outputTemp.temperature,size(c+1:b));
                    if abs(a-tC.delay)<1 && tC.rampActive
                        tC.device.startRamp(tC.rate);
                        tC.device.setTemp(tC.endTemp);
                        disp('ramp started')
                        pause(2)
                    end
                    if con_a_b(tempFactor,outputTemp.temperature<tC.endTemp,outputTemp.temperature>tC.endTemp) && (duration-a)>tC.delay
                        duration = a+tC.delay;
                    end
                end
                
                modeYData = con_a_b(strcmp(mode,'V'),current(1:ceil(length(current)/250):n1),voltage(1:ceil(length(voltage)/250):n1));
                if isgraphics(ax,'axes')
                    if input.Results.hold
                        hold on
                    else
                        hold off
                    end
                    
                    plot(ax,time(1:ceil(length(time)/250):n1),modeYData,'Color',ax.ColorOrder(1,:))
                    xlabel(ax,'Time (s)')
                    ylabel(ax,modeYLabel)
                    drawnow
                elseif isstruct(ax) && isfield(ax,'update')
                    ax.update(time(1:ceil(length(time)/250):n1),modeYData,'xlabel','Time (s)','ylabel',modeYLabel,'hold',input.Results.hold)
                end
                n1 = n1+1;
            end
            
            outputData.voltage = voltage(1:max(n1-1,1));
            outputData.current = current(1:max(n1-1,1));
            outputData.time    = time(1:max(n1-1,1));
            outputData.t0 = t0;
            if measureTemperature
                outputData.temperature = temperature(1:length(time));
            end
            
            device.getEventlogMessages();
            
            %# check if scan was aborted by user and adjust status
            if device.abortMeasurement
                device.abortMeasurement = 0;
                device.setOutputState(0);
                err = 88;
                return
            elseif device.abortAllMeasurements
                device.abortAllMeasurements = 0;
                device.setOutputState(0);
                err = 99;
                return
            else
                device.goIdle();
                device.setOutputState(0);
                err = 0;
                return
            end
        end
        
        function [outputData,err] = measureSteadyState(device,sourceV,duration,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'sourceV',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'duration',validationFcn('keithleyDuration',device.functionName));
            addOptional(input,'delay',0,validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',min(device.intRate*2,10),validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'plotHandle',0,@(x) (isnumeric(x) && x==0) || isgraphics(x,'axes') || (isstruct(x) && isfield(x,'update')));
            addParameter(input,'hold',0,validationFcn('boolean',device.functionName));
            addParameter(input,'track','mpp',@(x) any(validatestring(lower(x),{'mpp','voc','jsc'})));
            addParameter(input,'temperatureController',[]);
            parse(input,device,sourceV,duration,varargin{:})
            
            ax = input.Results.plotHandle;
            tC = input.Results.temperatureController;
            
            outputData = struct();
            outputData.power = 0;
            outputData.voltage = 0;
            outputData.current = 0;
            outputData.time    = 0;
            if device.abortMeasurement
                err = 88;
                return
            elseif device.abortAllMeasurements
                err = 99;
                return
            end
            
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            err = 0;

            if strcmpi(input.Results.track,'jsc')
                err = device.setTimeScan(0,input.Results.delay,input.Results.integrationRate);
                if err
                    return
                end
                
                [outputData,err] = device.measureTimeScan(duration,'plotHandle',ax,'hold',input.Results.hold,'temperatureController',tC);
                return
            elseif strcmpi(input.Results.track,'voc')
                err = device.setTimeScan(0,input.Results.delay,input.Results.integrationRate,'mode','I');
                if err
                    return
                end
                
                [outputData,err] = device.measureTimeScan(duration,'plotHandle',ax,'hold',input.Results.hold,'temperatureController',tC,'mode','I');
                return
            end
            
            %first data
            n1=1;
            power  = zeros(100000,1);
            voltage = zeros(100000,1);
            current = zeros(100000,1);
            time = zeros(100000,1);
            if ~isempty(tC)
                temperature = zeros(10000,1);
                tempFactor = tC.startTemp>tC.endTemp;
            end
            
            currentVolt = sourceV;
            step = 0.02;
            minstep = 0.001;
            maxstep = 0.05;
            
            %# reset timer in Keithley
            fprintf(device.connectionHandle,':SYST:TIME:RES');
            
            a = 0;
            while a<duration && ~device.abortMeasurement && ~device.abortAllMeasurements
                err = device.setSweepV(currentVolt-2*step,currentVolt+2*step,step,input.Results.delay,input.Results.integrationRate);
                if err
                    break;
                end
                [outputSweep,err] = device.measureSweepV();
                if err == 88 || err == 99
                    break;
                end
                
                %# calculate power of measurement and find max current powerpoint and index for next voltage
                sweepPower = abs(outputSweep.current.*outputSweep.voltage);
                switch input.Results.track
                    case 'mpp'
                        [power(n1),index] = max(sweepPower);
                    	current(n1) = outputSweep.current(index);
                    case 'voc'
                        [current(n1),index] = min(abs(outputSweep.current));
                        power(n1) = sweepPower(index);
                end
                voltage(n1) = outputSweep.voltage(index);
                time(n1) = outputSweep.time(index);
                a = time(n1)-time(1);
                
                %# temperature controll
                if ~isempty(tC)
                    outputTemp = tC.device.getTemp();
                    temperature(n1) = outputTemp.temperature;
                    if abs(a-tC.delay)<1 && tC.rampActive
                        tC.device.startRamp(tC.rate);
                        tC.device.setTemp(tC.endTemp);
                        disp('ramp started')
                    end
                    if con_a_b(tempFactor,outputTemp.temperature<tC.endTemp,outputTemp.temperature>tC.endTemp) && (duration-a)>tC.delay
                        duration = a+tC.delay;
                    end
                end
                
                %# new center voltage for next loop
                currentVolt = voltage(n1); 
                
                %# abort measurement to not destroy solar cell
                if abs(currentVolt) > 1.2 
                    err = 77;
                    break;
                end
                
                %# adjust scan range
                if index == 1 || index == 5
                    step = min(1.5*step,maxstep);
                else
                    step = max(round(step/4,3),minstep);
                end
                
                if isgraphics(ax,'axes')
                    if input.Results.hold
                        hold on
                    else
                        hold off
                    end
                    
                    switch input.Results.track
                        case 'mpp'
                            plot(ax,time(1:ceil(n1/250):n1),power(1:ceil(n1/250):n1),'Color',ax.ColorOrder(2,:))
                            xlabel(ax,'Time (s)')
                            ylabel(ax,'Maximum Power (mW)')
                        case 'voc'
                            plot(ax,time(1:ceil(n1/250):n1),voltage(1:ceil(n1/250):n1),'Color',ax.ColorOrder(3,:))
                            xlabel(ax,'Time (s)')
                            ylabel(ax,'V_{OC} (V)')
                    end
                    drawnow
                elseif isstruct(ax) && isfield(ax,'update')
                    switch input.Results.track
                        case 'mpp'
                            ax.update(time(1:ceil(n1/250):n1),power(1:ceil(n1/250):n1),'xlabel','Time (s)','ylabel','Maximum Power (mW)','hold',input.Results.hold)
                        case 'voc'
                            ax.update(time(1:ceil(n1/250):n1),voltage(1:ceil(n1/250):n1),'xlabel','Time (s)','ylabel','V_{OC} (V)','hold',input.Results.hold)
                    end
                    
                end                
                n1 = n1 + 1;
            end
            device.setOutputState(0);
            
            outputData.power  = abs(power(1:max(n1-1,1)));
            outputData.voltage = abs(voltage(1:max(n1-1,1)));
            outputData.current = abs(current(1:max(n1-1,1)));
            outputData.time = time(1:max(n1-1,1));
            if ~isempty(tC)
                outputData.temperature = temperature(1:max(n1-1,1));
            end
        end
        
        function [outputData,err] = measureTimePointV(device,sourceV,duration,delay,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'sourceV',validationFcn('keithleySetMultV',device.functionName));
            addRequired(input,'duration',validationFcn('keithleyDelayMult',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',device.intRate,validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'mode','V',validationFcn('keithleyMode',device.functionName));
            addParameter(input,'lightControl',[]);
            addParameter(input,'plotHandle',0,@(x) (isnumeric(x) && x==0) || isgraphics(x,'axes') || (isstruct(x) && isfield(x,'update')));
            parse(input,device,sourceV,duration,delay,varargin{:})
            
            mode = input.Results.mode;
            
            lC = input.Results.lightControl;
            if isfield(lC,'device')
                lC.state = lC.customLightMeasurement(lC.cLightList{lC.n},1);
                lC.duration = lC.customLightMeasurement(lC.cLightList{lC.n},2);
                lC.device.newList(lC.duration,lC.state); %Light control unit (shutter) should have implemented a function list, which accepts state and duration as numeric array
            end
            
            ax = input.Results.plotHandle;
            
            outputData = struct();
            outputData.voltage = 0;
            outputData.current = 0;
            outputData.time    = 0;
            if device.abortMeasurement
                err = 88;
                return
            elseif device.abortAllMeasurements
                err = 99;
                return
            end
            
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            err = device.setTimeScan(input.Results.sourceV(1),input.Results.delay,input.Results.integrationRate,'mode',mode);
            if err
                return
            end

            if isfield(lC,'device')
                % if light control is activated, start now
                err = lC.device.start();
%                 if err
%                     return
%                 end
            end
            [outputData,err] = device.measureTimeScan(input.Results.duration(1),'plotHandle',ax,'mode',mode);
            if err
                return
            end
            
            for n1 = 2:length(input.Results.sourceV)
                if input.Results.duration(n1)>0
                    err = device.updateTimeScan(input.Results.sourceV(n1),input.Results.delay,input.Results.integrationRate,'mode',mode);
                    if err
                        return
                    end
                    
                    [outputDataTemp,err] = device.measureTimeScan(input.Results.duration(n1),'plotHandle',ax,'hold',1,'mode',mode);
                    if err
                        return
                    end
                    
                    outputData.voltage = [outputData.voltage;outputDataTemp.voltage];
                    outputData.current = [outputData.current;outputDataTemp.current];
                    outputData.time = [outputData.time;outputDataTemp.time];
                    outputData.t0 = [outputData.t0;outputDataTemp.t0];
                end
            end
            device.setOutputState(0);
            if isfield(lC,'device')
                err = lC.device.close();
                err = lC.device.clear();
%                 if err
%                     return
%                 end
            end
            outputData.time = outputData.time-outputData.time(1);
            
            %# check if scan was aborted by user and adjust status
            if device.abortMeasurement
                device.setOutputState(0);
                device.abortMeasurement = 0;
                err = 88;
                return
            elseif device.abortAllMeasurements
                device.setOutputState(0);
                device.abortAllMeasurements = 0;
                err = 99;
                return
            else
                err = 0;
                return
            end
        end
        
        function [outputData,err] = measureTimeSweepV(device,start,stop,step,duration,delay,varargin)
            input = inputParser;
            addRequired(input,'device')
            addRequired(input,'start',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'stop',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'step',validationFcn('keithleySetStepV',device.functionName));
            addRequired(input,'duration',validationFcn('keithleyDuration',device.functionName));
            addRequired(input,'delay',validationFcn('keithleyDelay',device.functionName));
            addOptional(input,'integrationRate',min(device.intRate*2,10),validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'plotHandle',0,@(x) (isnumeric(x) && x==0) || isgraphics(x,'axes') || (isstruct(x) && isfield(x,'update')));
            parse(input,device,start,stop,step,duration,delay,varargin{:})
            
            ax = input.Results.plotHandle;
            
            outputData = struct();
            outputData.voltage = 0;
            outputData.current = 0;
            outputData.time    = 0;
            if device.abortMeasurement
                err = 88;
                return
            elseif device.abortAllMeasurements
                err = 99;
                return
            end
            
            if ~device.connected
                err = 1;
                errordlg(sprintf(getErrorMessage('deviceNotConnected'),device.defaultID))
                return
            end
            
            start = input.Results.start;
            stop = input.Results.stop;
            step = input.Results.step;
            duration = input.Results.duration;
            delay = input.Results.delay;
            integrationRate = input.Results.integrationRate;
            
            err = device.setTimeScan(start,delay,integrationRate);
            if err
                return
            end
            
            for sourceV=start:step:stop
                err = device.updateTimeScan(sourceV,delay,integrationRate);
                if err
                    return
                end
                
                if start==sourceV
                    [outputData,err] = device.measureTimeScan(duration,'plotHandle',ax);
                    t0 = outputData.time(1);
                else
                    [outputDataTemp,err] = device.measureTimeScan(duration,'plotHandle',ax,'hold',1);
                    outputData.voltage = [outputData.voltage;outputDataTemp.voltage];
                    outputData.current = [outputData.current;outputDataTemp.current];
                    outputData.time = [outputData.time;outputDataTemp.time-t0];
                end
                
                if err == 88 || err == 99
                    return
                end
            end
            device.setOutputState(0);
        end
    end
    methods (Access=private) %# low level device functions
        function startStarWars(device)
            fprintf(device.connectionHandle,'beeper.beep(0.4,500)');
            fprintf(device.connectionHandle,'delay(0.3)');
            fprintf(device.connectionHandle,'beeper.beep(0.4,500)');
            fprintf(device.connectionHandle,'delay(0.3)');
            fprintf(device.connectionHandle,'beeper.beep(0.4,500)');
            fprintf(device.connectionHandle,'delay(0.3)');
            fprintf(device.connectionHandle,'beeper.beep(0.4,400)');
            fprintf(device.connectionHandle,'delay(0.2)');
            fprintf(device.connectionHandle,'beeper.beep(0.2,600)');
            fprintf(device.connectionHandle,'delay(0.1)');
            fprintf(device.connectionHandle,'beeper.beep(0.3,500)');
            fprintf(device.connectionHandle,'delay(0.3)');
            fprintf(device.connectionHandle,'beeper.beep(0.4,400)');
            fprintf(device.connectionHandle,'delay(0.2)');
            fprintf(device.connectionHandle,'beeper.beep(0.2,600)');
            fprintf(device.connectionHandle,'delay(0.1)');
            fprintf(device.connectionHandle,'beeper.beep(0.3,500)');
            fprintf(device.connectionHandle,'delay(0.8)');

            fprintf(device.connectionHandle,'beeper.beep(0.4,600)');
            fprintf(device.connectionHandle,'delay(0.3)');
            fprintf(device.connectionHandle,'beeper.beep(0.4,600)');
            fprintf(device.connectionHandle,'delay(0.3)');
            fprintf(device.connectionHandle,'beeper.beep(0.4,600)');
            fprintf(device.connectionHandle,'delay(0.3)');
            fprintf(device.connectionHandle,'beeper.beep(0.4,635)');
            fprintf(device.connectionHandle,'delay(0.2)');
            fprintf(device.connectionHandle,'beeper.beep(0.2,500)');
            fprintf(device.connectionHandle,'delay(0.1)');
            fprintf(device.connectionHandle,'beeper.beep(0.3,400)');
            fprintf(device.connectionHandle,'delay(0.3)');
            fprintf(device.connectionHandle,'beeper.beep(0.4,375)');
            fprintf(device.connectionHandle,'delay(0.2)');
            fprintf(device.connectionHandle,'beeper.beep(0.2,600)');
            fprintf(device.connectionHandle,'delay(0.1)');
            fprintf(device.connectionHandle,'beeper.beep(0.3,500)');
            fprintf(device.connectionHandle,'delay(0.4)');
        end
    
        function makeBuffer(device,name,size,style)
            %name: string
            %size: >10
            %style: buffer.STYLE_COMPACT, buffer.STYLE_STANDARD, buffer.STYLE_FULL
            fprintf(device.connectionHandle,[name,' = buffer.make(',num2str(size),',',style,')']);
        end
        
        function deleteBuffer(device,name)
            fprintf(device.connectionHandle,['buffer.delete(',name,')']);
        end
        
        function updateBufferSize(device,name,size)
            fprintf(device.connectionHandle,[name,'.capacity = ',num2str(size)]);
        end
        
        function clearBuffer(device,name)
            fprintf(device.connectionHandle,[name,'.clear()']);
        end
        
        function setBufferFillmode(device,name,mode)
            % mode: buffer.FILL_ONCE or 0, buffer.FILL_CONTINUOUS or 1
            fprintf(device.connectionHandle,[name,'.fillmode = ',mode]);
        end
        
        function getBufferValues(device,name)
            test = query(device.connectionHandle,['print(',name,'.n)'])
        end
        
        function getBufferReadings(device,name,number)
            test = query(device.connectionHandle,['print(',name,'.formattedreadings[',num2str(number),'])'])
        end
        
        function delay(device,time)
            fprintf(device.connectionHandle,['delay(',num2str(time),')']);
        end
        
        function clearEventlog(device)
            fprintf(device.connectionHandle,'eventlog.clear()');
        end
        
		function eventlogCount = getEventlogCount(device)
            eventlogCount = str2double(query(device.connectionHandle,'print(eventlog.getcount())'));
        end
        
        function eventlog = readEventlog(device)
            eventlog = query(device.connectionHandle,'print(eventlog.next())');
        end
        function clearStatus(device)
            fprintf(device.connectionHandle,'status.clear()');
        end
        
        %Clears any pending command triggers.
        function clearTrigger(device)
            fprintf(device.connectionHandle,'trigger.clear()');
        end
        
        %Stops all trigger model commands on the instrument.
        function clearTriggerModel(device)
            fprintf(device.connectionHandle,'trigger.model.abort()');
        end
        
        %Starts the trigger model.
        function initTriggerModel(device)
            fprintf(device.connectionHandle,'trigger.model.initiate()');
        end
        
        %Loads a predefined trigger model configuration that does a specific number of measurements.
        function loadTriggerModeSimpleLoop(device,varargin)
            input = inputParser;
            addRequired(input,'device');
            addOptional(input,'bufferName','defbuffer1');
            addParameter(input,'delay',0);
            parse(input,device,varargin{:});
            
            fprintf(device.connectionHandle,['trigger.model.load("SimpleLoop",', num2str(input.Results.delay),',', input.Results.bufferName,')']);
        end
        
        %Loads a predefined trigger model configuration that makes continuous measurements for a specified amount of time
        function loadTriggerModeDuration(device,duration,varargin)
            fprintf(device.connectionHandle,['trigger.model.load("DurationLoop",', num2str(duration),',', num2str(delay),',', bufferName,')']);
        end
        
        %Loads a predefined trigger model configuration that makes continuous measurements for a specified amount of time
        function loadTriggerModeIO(device,digInLine,varargin)
            fprintf(device.connectionHandle,['trigger.model.load("LogicTrigger",', num2str(digInLine),',',num2str(digOutLine),',1,trigger.CLEAR_NEVER,', num2str(delay),',', bufferName,')']);
        end
        
        function getTriggerModelState(device)
            test = query(device.connectionHandle,'print(trigger.model.state())')
        end
        
        %# wait for trigger event
        function setTriggerWait(device)
            fprintf(device.connectionHandle,'trigger.wait()');
        end
        
        %# stop currently running script
        function exit(device)
            fprintf(device.connectionHandle,'exit()');
        end
        
        function setASCIIprecision(device,precision)
            %precision: 0 to 16
            fprintf(device.connectionHandle,['format.asciiprecision = ',num2str(precision)]);
        end
        
        function waitComplete(device)
            fprintf(device.connectionHandle,'opc()');
            fprintf(device.connectionHandle,'waitcomplete()');
        end
        
        function resetDevice(device)
            fprintf(device.connectionHandle,'reset()');
        end
        
        function resetTimer(device)
            fprintf(device.connectionHandle,'timer.cleartime()');
        end
        
        function time = getTime(device)
            time = str2double(query(device.connectionHandle,'print(timer.gettime())'));
        end
        
        function setAutoRange(device,varargin)
            input = inputParser;
            addRequired(input,'device');
            addParameter(input,'autoRange',1,validationFcn('boolean',device.functionName));
            addParameter(input,'range',2,validationFcn('keithleySetV',device.functionName));
            parse(input,device,varargin{:});

            autoRange = upper(con_on_off(input.Results.autoRange));
            range = input.Results.range;
            
            fprintf(device.connectionHandle,['smu.measure.autorange = smu.',autoRange]);
            if ~input.Results.autoRange
                fprintf(device.connectionHandle,['smu.measure.range = ',num2str(range)]);    % Select range by specifying the expected voltage 
            end
        end
        
        function setAutoRangeMax(device,value)
            %• Current: 1e-8 to 1 A
            %• Resistance: 2 to 200e6 
            %• Voltage: 0.02 to 200 V
            fprintf(device.connectionHandle,['smu.measure.autorangerangehigh = ',num2str(value)]);
        end
        
        function setAutoRangeMin(device,value)
            %• Current: 1e-8 to 1 A
            %• Resistance: 2 to 200e6 
            %• Voltage: 0.02 to 200 V
            fprintf(device.connectionHandle,['smu.measure.autorangerangelow = ',num2str(value)]);
        end
        
        function setAutoRangeSource(device,varargin)
            input = inputParser;
            addRequired(input,'device');
            addParameter(input,'autoRange',1,validationFcn('boolean',device.functionName));
            addParameter(input,'range',2,validationFcn('keithleySetV',device.functionName));
            parse(input,device,varargin{:});

            autoRange = upper(con_on_off(input.Results.autoRange));
            range = input.Results.range;
            
            fprintf(device.connectionHandle,['smu.source.autorange = smu.',autoRange]);
            if ~input.Results.autoRange
                fprintf(device.connectionHandle,['smu.source.range = ',num2str(range)]);
            end
        end
        
        function setAutoZero(device,varargin)
            input = inputParser;
            addRequired(input,'device');
            addOptional(input,'state',0,validationFcn('boolean',device.functionName));
            parse(input,device,varargin{:});
            
            state = upper(con_on_off(input.Results.state));
            fprintf(device.connectionHandle,['smu.measure.autozero.enable = smu.',state]);
        end
        
        function setAutoZeroOnce(device)
            fprintf(device.connectionHandle,'smu.measure.autozero.once()');
        end
            
        function setCount(device,value)
            fprintf(device.connectionHandle,['smu.measure.count = ',num2str(value)]);
        end
        
        function setIntegrationRate(device,integrationRate)
            %integrationRate: 0.01 to 10
            fprintf(device.connectionHandle,['smu.measure.nplc = ',num2str(integrationRate)]);
        end
        
        function setTerminals(device,terminals)
            % terminals: smu.TERMINALS_REAR, smu.TERMINALS_FRONT
            fprintf(device.connectionHandle,['smu.measure.terminals = ',terminals]);
        end
        
        function setProtectionLevel(device,varargin)
            input = inputParser;
            addRequired(input,'device');
            addOptional(input,'level',2,validationFcn('keithleyProtLevel',device.functionName));
            parse(input,device,varargin{:});
            %smu.PROTECT_x, where x is 2V, 5V, 10V, 20V, 40V, 60V, 80V, 100V, 120V, 140V, 160V, 180V, or NONE
            if (input.Results.level == 0)
                level = 'NONE';
            elseif (input.Results.level <= 2)
                level = '2V';
            elseif (input.Results.level <= 5)
                level = '5V';
            elseif (input.Results.level <= 10)
                level = '10V';
            elseif (input.Results.level <= 20)
                level = '20V';
            elseif (input.Results.level <= 40)
                level = '40V';
            elseif (input.Results.level <= 60)
                level = '60V';
            elseif (input.Results.level <= 80)
                level = '80V';
            elseif (input.Results.level <= 100)
                level = '100V';
            elseif (input.Results.level <= 120)
                level = '120V';
            elseif (input.Results.level <= 140)
                level = '140V';
            elseif (input.Results.level <= 160)
                level = '160V';
            elseif (input.Results.level <= 180)
                level = '180V';
            end
            fprintf(device.connectionHandle,['smu.source.protect.level = smu.PROTECT_',level]);
        end
        
        function setSourceLimit(device,mode,level)
            fprintf(device.connectionHandle,['smu.source.',mode,'limit.level',num2str(level)]);
        end
        
        function data = readData(device)
            data = query(device.connectionHandle,'smu.measure.read()');
        end
        
        function data = readBuffer(device,varargin)
            input = inputParser;
            addRequired(input,'device');
            addOptional(input,'buffer','defbuffer1');
			addParameter(input,'startIndex',1);
			addParameter(input,'endIndex',1);
            parse(input,device,varargin{:});
            
            data = query(device.connectionHandle,strrep(['printbuffer(',num2str(input.Results.startIndex),',',num2str(input.Results.endIndex),',bufferName.sourcevalues,bufferName.readings,bufferName.relativetimestamps)'],'bufferName',input.Results.buffer));
        end
        
        function err = setDelay(device, mode, varargin)
            input = inputParser;
            addRequired(input,'device');
            addRequired(input,'mode',validationFcn('mode',device.functionName));
            addOptional(input,'time',0,validationFcn('keithleyDelay',device.functionName));
            parse(input,device,mode,varargin{:});

            err = 0;
            try
                switch lower(mode)
                    case 'auto'
                        fprintf(device.connectionHandle,'smu.source.autodelay = smu.ON');                             % Enable auto settling (delay) time.
                    case 'manual'
                        fprintf(device.connectionHandle,'smu.source.autodelay = smu.OFF');                            % Disable auto settling (delay) time.
                        fprintf(device.connectionHandle,['smu.source.delay = ', num2str(input.Results.time)]);      % Specify settling (delay) time (in sec): 0 to 9999.999.
                end
            catch E
                errordlg(['Error in function ',device.functionName]);
                disp(E.message);
                err = 1;
                return
            end
        end
        
        function err = setSourceMode(device, mode)
            input = inputParser;
            addRequired(input,'device');
            addRequired(input,'mode',validationFcn('keithleyMode',device.functionName));
            parse(input,device,mode);

            % Select source mode (VOLTage, or CURRent).
            err = 0;
            try
                switch mode
                    case 'I'
                        fprintf(device.connectionHandle,'smu.source.func = smu.FUNC_DC_CURRENT');
                    case 'V'
                        fprintf(device.connectionHandle,'smu.source.func = smu.FUNC_DC_VOLTAGE');
                end
            catch E
                errordlg(['Error in function ',device.functionName]);
                disp(E.message);
                err = 1;
                return
            end
        end
        
        function err = setSourceVoltage(device, varargin)
            input = inputParser;
            addRequired(input,'device');
            addOptional(input,'level',0,validationFcn('keithleySetV',device.functionName));
            parse(input,device, varargin{:});

            err = 0;
            try
                fprintf(device.connectionHandle,['smu.source.level =',num2str(input.Results.level)]);
            catch E
                errordlg(['Error in function ',device.functionName]);
                disp(E.message);
                err = 1;
                return
            end
        end
        
        function err = setSourceCurrent(device, varargin)
            input = inputParser;
            addRequired(input,'device');
            addOptional(input,'level',0,validationFcn('keithleySetI',device.functionName));
            parse(input,device, varargin{:});

            err = 0;
            try
                fprintf(device.connectionHandle,['smu.source.level =',num2str(input.Results.level)]);
            catch E
                errordlg(['Error in function ',device.functionName]);
                disp(E.message);
                err = 1;
                return
            end
        end
        
        function err = setSense(device, mode, varargin)
            input = inputParser;
            addRequired(input,'device');
            addRequired(input,'mode',validationFcn('keithleyMode',device.functionName));
            addParameter(input,'integrationRate',1,validationFcn('keithleyIntegrationRate',device.functionName));
            addParameter(input,'complianceLevel',2,validationFcn('keithleySetV',device.functionName));
            addParameter(input,'autoRange',1,validationFcn('boolean',device.functionName));
            addParameter(input,'range',2,validationFcn('keithleySetV',device.functionName));
            parse(input,device,mode,varargin{:});

            integrationRate = input.Results.integrationRate;
            complianceLevel = con_a_b(strcmpi(mode,'V'),min(input.Results.complianceLevel,20),min(input.Results.complianceLevel,1));
            range = input.Results.range;
            
            %smu.measure.sense = smu.SENSE_4WIRE %smu.SENSE_2WIRE
            %smu.measure.offsetcompensation = smu.ON
            
            %range
            %Current: 1 nA to 1 A
            %Resistance: 20 to 200 MOhm
            %Voltage: 0.02 to 200 V

            err = 0;
            switch upper(mode)
                case 'I'
                    setMode = 'smu.FUNC_DC_CURRENT'; %smu.FUNC_DC_CURRENT, smu.FUNC_DC_VOLTAGE, smu.FUNC_RESISTANCE
                    
%                     fprintf(device.connectionHandle,':SENS:FUNC:CONC ON');                   % Enable or disable ability to measure more than one function simultaneously. When disabled, volts function is enabled.                   
                case 'V'
                    setMode = 'smu.FUNC_DC_VOLTAGE';
                case 'R'
                    setMode = 'smu.FUNC_RESISTANCE';
            end
            fprintf(device.connectionHandle,['smu.measure.func = ',setMode]);
            device.setIntegrationRate(integrationRate);
            device.setProtectionLevel(complianceLevel);
            device.setAutoRange('autoRange',input.Results.autoRange,'range',range);
            device.setAutoRangeSource('autoRange',input.Results.autoRange,'range',range);
        end
        
        function err = setSingle(device, mode, varargin)
            input = inputParser;
            addRequired(input,'device');
            addRequired(input,'mode',validationFcn('keithleyMode',device.functionName));
            addOptional(input,'level',0,validationFcn('keithleySetV',device.functionName));
			addParameter(input,'points',1,validationFcn('keithleyDuration',device.functionName));
            parse(input,device.connectionHandle, mode, varargin{:});

            err = 0;
            try
                fprintf(device.connectionHandle,['trigger.model.load("SimpleLoop",',num2str(input.Results.points),')']);
                switch upper(mode)
                    case 'I'
                        fprintf(device.connectionHandle,'smu.source.func = smu.FUNC_DC_CURRENT');
                        fprintf(device.connectionHandle,'smu.source.range = 1');
                    case 'V'
                        fprintf(device.connectionHandle,'smu.source.func = smu.FUNC_DC_VOLTAGE');
                        fprintf(device.connectionHandle,'smu.source.range = 20');
                end

                fprintf(device.connectionHandle,['smu.source.level = ',num2str(input.Results.level)]);
            catch E
                errordlg(['Error in function ',device.functionName]);
                disp(E.message);
                err = 1;
                return
            end
        end
        
        function err = setSweep(device, mode, start, stop, step, varargin)
            input = inputParser;
            addRequired(input,'device');
            addRequired(input,'mode',validationFcn('keithleyMode',device.functionName));
            addRequired(input,'start',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'stop',validationFcn('keithleySetV',device.functionName));
            addRequired(input,'step',validationFcn('keithleySetStepV',device.functionName));
            addParameter(input,'delay',0,validationFcn('keithleyDelay',device.functionName));
            addParameter(input,'spacing','LIN'); % there is also a log sweep function, if needed #ToDo
            addParameter(input,'bufferName','defbuffer1');
            parse(input,device,mode,start,stop,step,varargin{:})

            err = 0;
            try
                fprintf(device.connectionHandle,'trigger.model.load("SimpleLoop", 1)');
                switch upper(mode)
                    case 'I'
                        fprintf(device.connectionHandle,'smu.source.func = smu.FUNC_DC_CURRENT');
%                         fprintf(device.connectionHandle,'smu.source.range = 1');
                        fprintf(device.connectionHandle,['smu.source.sweeplinearstep("CurrLinSweep",', num2str(start),',',num2str(stop),',',num2str(step),',',num2str(input.Results.delay),',1,smu.RANGE_BEST,smu.ON,smu.OFF,',input.Results.bufferName,')']);
                    case 'V'
                        fprintf(device.connectionHandle,'smu.source.func = smu.FUNC_DC_VOLTAGE');
%                         fprintf(device.connectionHandle,'smu.source.range = 20');
                        fprintf(device.connectionHandle,['smu.source.sweeplinearstep("VoltLinSweep",', num2str(start),',',num2str(stop),',',num2str(step),',',num2str(input.Results.delay),',1,smu.RANGE_BEST,smu.ON,smu.OFF,',input.Results.bufferName,')']);
                end
            catch E
                errordlg(['Error in function ',device.functionName]);
                disp(E.message);
                err = 1;
                return
            end
        end
        
        function err = setList(device, mode, list, varargin)
            input = inputParser;
            addRequired(input,'device');
            addRequired(input,'mode',validationFcn('keithleyMode',device.functionName));
            addRequired(input,'list',validationFcn('keithleyList',device.functionName));
            addParameter(input,'delay',0,validationFcn('keithleyDelay',device.functionName));
            parse(input,device,mode,list,varargin{:})

            err = 0;
            try
                switch upper(mode)
                    case 'I'
                        fprintf(device.connectionHandle,'smu.source.configlist.create("ListSweep")');
                        
                        fprintf(device.connectionHandle,'smu.source.func = smu.FUNC_DC_CURRENT');
                        fprintf(device.connectionHandle,'smu.source.range = 1');
                        
                    case 'V'
                        fprintf(device.connectionHandle,'smu.source.configlist.create("ListSweep")');
                        
                        fprintf(device.connectionHandle,'smu.source.func = smu.FUNC_DC_VOLTAGE');
                        fprintf(device.connectionHandle,'smu.source.range = 20');
                end
                %# send list with source settings to device
                for n0 = 1:length(list)
                    fprintf(device.connectionHandle,['smu.source.level = ',num2str(list(n0))]);
                    fprintf(device.connectionHandle,'smu.source.configlist.store("ListSweep")');
                end

                %smu.source.sweeplist(configListName, index, delay)
                fprintf(device.connectionHandle,['smu.source.sweeplist("ListSweep", 1,', num2str(input.Results.delay),')']);
            catch E
                errordlg(['Error in function ',device.functionName]);
                disp(E.message);
                err = 1;
                return
            end
        end
        
        function err = toggleOutput(device, varargin)
            input = inputParser;
            addRequired(input,'device');
            addOptional(input,'state',0,validationFcn('boolean',device.functionName));
            parse(input,device,varargin{:});
            
            state = upper(con_on_off(input.Results.state));

            err = 0;
            try
                fprintf(device.connectionHandle,['smu.source.output = smu.',state]);
            catch E
                errordlg(['Error in function ',device.functionName]);
                disp(E.message);
                err = 1;
                return
            end
        end
        
        function clearDevice(device)
            device.exit();
            
            %# clear Trigger events
            device.clearTriggerModel();
            device.clearTrigger();
        end
        
        function device = initialize(devTemp, connectionType, port)
            input = inputParser;
            addRequired(input,'devTemp');
            addRequired(input,'connectionType',validationFcn('connectionType',devTemp.functionName))
            addRequired(input,'port',validationFcn('generalPort',devTemp.functionName));
            parse(input,devTemp,connectionType,port);

            switch lower(connectionType)
                case 'serial'
                    device = instrfind('Type', 'serial', 'Port', port, 'Tag', '');
                    if isempty(device)
                        device = serial(port, 'Terminator', {'CR','CR'}, 'BaudRate', 57600);
                    else
                        fclose(device);
                        device = device(1);
                    end

                    % Set the property values.
                    set(device, 'ByteOrder', 'littleEndian');
                    set(device, 'ErrorFcn', '');
                    set(device, 'InputBufferSize', 100000);
                    set(device, 'OutputBufferSize', 100000);
                    set(device, 'OutputEmptyFcn', '');
                    set(device, 'UserData', []);

                case 'gpib'
                    device = instrfind('Type', 'gpib', 'BoardIndex', 0, 'PrimaryAddress', port);
                    if isempty(device)
                        device = gpib('NI', 0, port);
                    else
                        fclose(device);
                        device = device(1);
                    end

                    % Set the property values.
                    set(device, 'BoardIndex', 0);
                    set(device, 'ByteOrder', 'littleEndian');
                    set(device, 'BytesAvailableFcnMode', 'eosCharCode');
                    set(device, 'CompareBits', 8);
                    set(device, 'EOIMode', 'on');
                    set(device, 'EOSCharCode', 'CR');
                    set(device, 'EOSMode', 'read&write');
                    set(device, 'ErrorFcn', '');
                    set(device, 'InputBufferSize', 100000);
                    set(device, 'OutputBufferSize', 100000);
                    set(device, 'OutputEmptyFcn', '');
                    set(device, 'SecondaryAddress', 0);
                    set(device, 'UserData', []);
            end

            %# set Terminator to device settings
            set(device, 'Timeout', 2);

            %# Connect to instrument object.
            fopen(device);

            fprintf(device,'reset()');                  % Return SourceMeter to GPIB defaults
            fprintf(device,'eventlog.clear()');         % Clears all event registers and Error Queue.
            fprintf(device,'status.clear()');

%             fprintf(device, ':SYST:BEEP:STAT OFF');
        end
    end
end