%% Real Time Data Streaming with Delsys SDK

% Copyright (C) 2020 Delsys, Inc.
%
% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files (the "Software"),
% to deal in the Software without restriction, including without limitation
% the rights to use, copy, modify, merge, publish, and distribute the
% Software, and to permit persons to whom the Software is furnished to do so,
% subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
% FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
% DEALINGS IN THE SOFTWARE.

function [] = DelsysTrignoControl_Trigger_Block_Acc(varargin)
%% DELSYSTRIGNOCONTROL streams data from the Delsys Trigno Control Utility
%
%   Optional inputs:
%       1) IP address of host PC running Trigno Control Utility
%       2) Run time (in seconds)
%
%   Optional outputs:
%       1) Data from EMG Data port
%       2) Data from AUX Data port
%       3) Data from Legacy EMG Data port
%       4) Data from Legacy AUX Data port


%% CONFIGURE TCP/IP CONNECTIONS
close all;
clear all hidden;
%IP of the host computer running the Trigno Control Utility + scheduled
%run time (in seconds)
if nargin == 0
    hostIP = 'localhost'; %'localhost'
    runTime = 60;
    type = 'statemachine'; %'gui'
elseif nargin == 1
    hostIP = varargin{1};
    runTime = 60;
    type = 'statemachine'; %'gui'
    
elseif nargin == 2
    hostIP = varargin{1};
    runTime = varargin{2};
    type = 'statemachine'; %'gui'
    
else
    hostIP = varargin{1};
    runTime = varargin{2};
    type = varargin{3};
end


connected = 0;
%Port configuration parameter
fcnMode = 'byte';
%         global commPort;
[commPort, numChannels,emgDataPort,auxDataPort,...
    legacyEmgDataPort,legacyAuxDataPort] = configureTCPIPconnection();
global plotHandlesEMG;
global emgDataArray;
global totalSamples;
global emgSampleCounter;
global emgPlotBuffer;
global downsampleRate;
global figureHandleEMG;
global dataType;
global samplesPerFrame;
global frameInterval;

% data = {};
global emgDataArrayToSave;
global emgSampleCounterToSave;
global blocknum;
global foldername;

global plotHandlesAUX;
global auxDataArray;
global auxDataArrayToSave;
global auxSampleCounter;
global auxPlotBuffer;
global figureHandleAUX;
global auxBytesToRead;
global auxSampleCounterToSave;

global tcpipServer;
global EMGStarted;
%% create gui
switch type
    case 'gui'
        S.fh = figure('units','pixels',...
            'position',[500 200 500 400],...
            'menubar','none',...
            'name','DelsysTrignoControl',...
            'numbertitle','off',...
            'resize','off');%,...
        %               'WindowKeyPressFcn',@StartClbk);
        
        % S.tx = uicontrol('style','text',...
        %     'units','pix',...
        %     'position',[10 55 180 40],...
        %     'string','start data collection',...
        %     'fontsize',23);
        S.connectTCPbutton = uicontrol('style','push',...
            'units','pix',...
            'position',[10 235 180 40],...
            'fontsize',14,...
            'string','SEND TCP',...
            'call',@sendTCP);
        
        S.readTCPbutton = uicontrol('style','push',...
            'units','pix',...
            'position',[10 190 180 40],...
            'fontsize',14,...
            'string','READ TCP',...
            'call',@readTCP);
        
        S.closeTCPbutton = uicontrol('style','push',...
            'units','pix',...
            'position',[10 145 180 40],...
            'fontsize',14,...
            'string','CLOSE TCP',...
            'call',@closeTCP);
        
        S.connectbutton = uicontrol('style','push',...
            'units','pix',...
            'position',[10 100 180 40],...
            'fontsize',14,...
            'string','CONNECT',...
            'call',@connectClbk);
        
        S.startbutton = uicontrol('style','push',...
            'units','pix',...
            'position',[10 55 180 40],...
            'fontsize',14,...
            'string','START',...
            'call',{@startClbk,commPort});
        
        S.stopbutton = uicontrol('style','push',...
            'units','pix',...
            'position',[10 10 180 40],...
            'fontsize',14,...
            'string','STOP',...
            'call',{@stopClbk,commPort});
        
        set(S.connectbutton,'Enable','on');
        set(S.startbutton,'Enable','off');
        set(S.stopbutton,'Enable','off');
        set(S.readTCPbutton,'Enable','off');
        set(S.closeTCPbutton,'Enable','on');
        EMGStarted = 0;
        
    case 'statemachine'
        
        StateMachine();
        
end

%%
% blocknum = 1;
    function StateMachine(varargin)
        EMGStarted = 0;
        pauseLength = 3.0; %seconds
        
        %connect emg
        connectClbk();
        state = 'waitForTCPIP';
        tcpopen = 0;
        message = '';
        
        while(1)
            if tcpopen
                ncount = tcpipServer.BytesAvailable;
                if ncount>0
                    message = readTCP()
                    flushinput(tcpipServer);
                    flushoutput(tcpipServer);
                else
                    message = '';
                end
            end
            % end
            
            switch message
                
                case 'socket is sent'
                    state = 'readyForBlock';
                    
                case 'start emg'
                    state = 'BlockStarted';
                    startClbk([],[],commPort);
                    
                case 'stop emg'
                    state = 'endBlock';
                    stopClbk([],[],commPort);
                    
                case 'end session'
                    %                     if EMGStarted == 0
                    state = 'endSession';
                    %                     end
            end
            
            switch state
                
                case 'waitForTCPIP'
                    
                    %open tcp
                    createTCP();
                    tcpopen = 1;
                    disp('tcp open');
                    fileLogname = sprintf('%s\\LogFile.txt',foldername);
                    saveLogFile = fopen(fileLogname,'w');
                    state = 'readyForCommunication';
                    
                case  'readyForCommunication'
                    disp('readyForCommunication')
                    
                case 'readyForBlock'
                    %                     switch message
                    %                         case 'end session'
                    %                              state = 'endSession';
                    %                         otherwise
                    sendTCP('readyForBlock');
                    state = 'waitForBlock';
                    
                    disp('readyForBlock')
                    
                    %                     end
                case 'waitForBlock'
                    %                     message
                    if length(message) == 11
                        blocknum = str2num(message(11));
                        trialmessage = message(1:9);
                    elseif length(message) == 12
                        blocknum = str2num(message(11:12));
                        trialmessage = message(1:9);
                    else
                        trialmessage = '';
                    end
                    
                    switch trialmessage
                        case 'Block num'
                            
                            filename = sprintf('%s\\Trial_%02i.mat',foldername,blocknum);
                                                        filenameAux = sprintf('%s\\Trial_Acc_%02i.mat',foldername,blocknum);

                            %                             saveDataFile = fopen(filename,'w');
                            
                            tt = datetime;
                            t = datestr(tt,'HH-MM-ss.FFF');
                            
                            fprintf(saveLogFile,'trial %02i started %s\t',blocknum,t);
                            
                            %                              flushinput(commPort);
                            %         flushoutput(commPort);
                            %arm the system
                            %                             tic
                            startAcquisition(commPort);
                            
                            while commPort.BytesAvailable == 0
                                disp('wait for bytes')
                                
                            end
                            %                             start_toc(blocknum) = toc
                            startFlag = fread(commPort,commPort.BytesAvailable);
                            startFlag = strtrim(char(startFlag'))
                            if strcmp(startFlag,'OK')
                                sendTCP('BlockReceived');
                                state = 'waitForStartRecording';
                                drawnow;
                            end
                    end
                    
                case 'waitForStartRecording'
                    updateEmgPlot(plotHandlesEMG,emgDataArray,emgSampleCounter,emgPlotBuffer,downsampleRate);
                    updateAuxPlot(plotHandlesAUX,auxDataArray,auxSampleCounter,auxPlotBuffer,downsampleRate);
                    
                    disp('waitForStart')
                    
                    
                case 'BlockStarted'
                    %         start emg recording
                    
                    %                     if EMGStarted
                    
                    disp('EMGStarted')
                    updateAuxPlot(plotHandlesAUX,auxDataArray,auxSampleCounter,auxPlotBuffer,downsampleRate);
                    
                    updateEmgPlot(plotHandlesEMG,emgDataArray,emgSampleCounter,emgPlotBuffer,downsampleRate);
                    emgDataArrayToSave = zeros(totalSamples(1),numChannels(1),dataType);
                    emgSampleCounterToSave = 0;
                    auxDataArrayToSave = zeros(totalSamples(2),numChannels(2),dataType);
                    auxSampleCounterToSave = 0;
                    
                    sendTCP('EMGStarted');
                    tt = datetime;
                    t = datestr(tt,'HH-MM-ss.FFF');
                    
                    fprintf(saveLogFile,'EMGStarted %s\t',t);
                    
                    disp('EMGStarted sent')
                    
                    state =  'waitForStop';
                    %                     end
                case 'waitForStop'
                    updateEmgPlot(plotHandlesEMG,emgDataArray,emgSampleCounter,emgPlotBuffer,downsampleRate);
                    updateAuxPlot(plotHandlesAUX,auxDataArray,auxSampleCounter,auxPlotBuffer,downsampleRate);
                    
                case 'endBlock'
                    %                     data{blocknum} = emgDataArrayToSave;
                    %                     fprintf(saveDataFile,'%6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\t %6.10f\n',emgDataArrayToSave);
                    % tic
                    % dlmwrite(filename,emgDataArrayToSave);
                    % filename
                    %                     tic
                    stopAcquisition(commPort);
                    
                    while commPort.BytesAvailable == 0
                        disp('wait for bytes')
                        
                    end
                    %                     stop_toc(blocknum) = toc
                    
                    stopFlag = fread(commPort,commPort.BytesAvailable);
                    stopFlag = strtrim(char(stopFlag'))
                    if strcmp(stopFlag,'OK')
                        state =  'readyForBlock';
                        
                        %                         tic
                        save(filename,'emgDataArrayToSave');
                        save(filenameAux,'auxDataArrayToSave');
                        disp(filenameAux);
                        %                         save_toc(blocknum) = toc
                        
                        % toc
                        emgDataArrayToSave = zeros(totalSamples(1),numChannels(1),dataType);
                        emgSampleCounterToSave = 0;
                        auxDataArrayToSave = zeros(totalSamples(2),numChannels(2),dataType);
                        auxSampleCounterToSave = 0;
                        
                        EMGStarted = 0;
                        tt = datetime;
                        t = datestr(tt,'HH-MM-ss.FFF');
                        %                     tic
                        fprintf(saveLogFile,'endBlock %s\n',t);
                        %                     toc
                        disp('block is over');
                        sendTCP('EMGStopped');
                        
                    end
                    %                                         fclose(saveDataFile);
                    
                case 'endSession'
                    closeTCP();
                    fclose(saveLogFile);
                    break;
            end
        end
    end
%%
    function createTCP(varargin)
        %enable communication with Unity
        %same computer as unity
        %                 tcpipServer = tcpip('127.0.0.1',55000,'NetworkRole','Server','Timeout',100);
        % from optitrack computer to unity
        tcpipServer = tcpip('192.168.104.144',55000,'NetworkRole','Server','Timeout',100);
        fopen(tcpipServer);
        tcpipServer.ReadAsyncMode = 'continuous';
        switch type
            case 'gui'
                set(S.readTCPbutton,'Enable','on');
        end
    end

    function sendTCP(a)
        
        fwrite(tcpipServer,a);
        sprintf(' %s message sent',a);
        
    end
    function message = readTCP(varargin)
        
        ncount = tcpipServer.BytesAvailable;
        if ncount>0
            rawData = fread(tcpipServer,ncount,'char');
            
            for i=1:ncount
                rawwData(i)= char(rawData(i));
                
            end
        else
            rawwData = '';
        end
        message = rawwData;
        
    end

    function closeTCP(varargin)
        stopAcquisition(commPort);
        
        fclose(tcpipServer);
        
        closeConnections(figureHandleEMG,figureHandleAUX,0,emgDataPort,auxDataPort,...
            legacyEmgDataPort,legacyAuxDataPort,commPort);
    end

    function connectClbk(varargin)
        %         createTCP();
        %                 [commPort, numChannels] = configureTCPIPconnection();
        
        [figureHandleEMG,figureHandleAUX] = buildFigure(emgDataPort,auxDataPort,...
            legacyEmgDataPort,legacyAuxDataPort,commPort);
        [samplesPerFrame, frameInterval] = openConnection();
        [emgDataArray,emgBytesToRead,emgSampleCounter] = configureDataManager(samplesPerFrame, frameInterval);
        [emgPlotBuffer,downsampleRate] = configureDataport(emgBytesToRead,samplesPerFrame,frameInterval,fcnMode);
        configureAuxDataport(fcnMode);
        %         configureLegacyDataport();
        %         configureLegacyAuxDataport();
        
        %         pause(5);
        plotHandlesEMG = buildPlotAxes(figureHandleEMG,emgPlotBuffer);
        plotHandlesAUX = buildPlotAuxAxes(figureHandleAUX,auxPlotBuffer);
        
        dataStream();
        %Extended pause to allow axes to render before streaming
        %pause(5);
        %         updateEmgPlot(plotHandlesEMG,emgDataArray,emgSampleCounter,emgPlotBuffer,downsampleRate);
        
        %         startAcquisition(commPort);
        %         drawnow;
        %         updateEmgPlot(plotHandlesEMG,emgDataArray,emgSampleCounter,emgPlotBuffer,downsampleRate);
        
        disp('connected')
        
        connected = 1;
        
        switch type
            case 'gui'
                set(S.startbutton,'Enable','on');
                set(S.connectbutton,'Enable','off');
        end
        
        %create folder for saving
        tt = datetime;
        t = datestr(tt,'yyyymmdd-HHMMSS');
        foldername = sprintf('%s',t);
        mkdir(foldername);
        
        
    end
    function startClbk(~,~,commPort)
        
        flushinput(commPort);
        flushinput(emgDataPort);
        flushoutput(commPort);
        flushoutput(emgDataPort);
        %         startAcquisition(commPort);
        
        disp('pressed start')
        
        
        switch type
            case 'gui'
                %set(S.tx,'String', get(S.pb,'String'))
                set(S.stopbutton,'Enable','on');
                set(S.startbutton,'Enable','off');
        end
    end
    function stopClbk(~,~,commPort)
        
        %         stopAcquisition(commPort);
        flushinput(commPort);
        flushinput(emgDataPort);
        flushoutput(commPort);
        flushoutput(emgDataPort);
        
        %Delete timer, delete figure, close connections
        %         closeConnections(figureHandleEMG,0,emgDataPort,auxDataPort,...
        %             legacyEmgDataPort,legacyAuxDataPort,commPort);
        
        switch type
            case 'gui'
                set(S.stopbutton,'Enable','off');
                set(S.startbutton,'Enable','on');
        end
        disp('pressed stop')
    end
%%
    function [commPort, numChannels,emgDataPort,auxDataPort,...
            legacyEmgDataPort,legacyAuxDataPort] = configureTCPIPconnection()
        %TCP connection to Command port:
        %   RECEIVES: Control commands & queries
        %   SENDS: Replies to control commands & queries
        commPort = tcpip(hostIP, 50040);
        
        %Channel numbers of data ports
%         numChannels = ...
%             [16;... %EMG Data port
%             9;... %AUX Data port
%             16;...  %Legacy EMG Data port
%             48];    %Legacy AUX Data port
         numChannels = ...
            [16;... %EMG Data port
            144;... %AUX Data port
            16;...  %Legacy EMG Data port
            48];    %Legacy AUX Data port
        
        %Timer for data acquisition
        % timerPeriod = 0.1;
        % timerCalls = ceil(runTime / timerPeriod);
        % t = timer(...
        %     'Period', timerPeriod,...
        %     'ExecutionMode', 'fixedSpacing',...
        %     'TasksToExecute', timerCalls,...
        %     'TimerFcn', @updateEmgPlot,...
        %     'StartFcn', {@startAcquisition, commPort},...
        %     'StopFcn', {@stopAcquisition, commPort});
        
        %TCP connection to EMG Data port (16 channels):
        %   RECEIVES: N/A
        %   SENDS: EMG and primary non-EMG data from all connected
        %   Trigno sensors
        emgDataPort = tcpip(hostIP, 50043);
        emgDataPort.InputBufferSize = 6400;
        
        %TCP connection to AUX Data port (144 channels):
        %   RECEIVES: N/A
        %   SENDS: Auxiliary non-EMG data from all connected Trigno
        %   sensors
        auxDataPort = tcpip(hostIP, 50044);
        auxDataPort.InputBufferSize = 6400;
        
        %TCP connection to Legacy EMG Data port (16 channels):
        %   RECEIVES: N/A
        %   SENDS: EMG and primary non-EMG data from connected Trigno
        %   sensors with 4 or fewer channels
        legacyEmgDataPort = tcpip(hostIP, 50041);
        legacyEmgDataPort.InputBufferSize = 6400;
        
        %TCP connection to Legacy AUX Data port (48 channels):
        %   RECEIVES: N/A
        %   SENDS: Auxiliary non-EMG data from connected Trigno sensors
        %   with 4 or fewer channels
        legacyAuxDataPort = tcpip(hostIP, 50042);
        legacyAuxDataPort.InputBufferSize = 6400;
        
    end


%% BUILD FIGURE AND AXES FOR PLOTTING INCOMING DATA
    function [figureHandleEMG,figureHandleAUX] = buildFigure(emgDataPort,auxDataPort,...
            legacyEmgDataPort,legacyAuxDataPort,commPort)
        
        
        %Build figure with callback for closing
        figureHandleEMG = figure('Name','EMG Data','Numbertitle','off',...
            'Position',[50 200 750 750] ,...
            'CloseRequestFcn',{@closeConnections,emgDataPort,auxDataPort,...
            legacyEmgDataPort,legacyAuxDataPort,commPort});
        
        %Build figure with callback for closing
        figureHandleAUX = figure('Name','ACC Data','Numbertitle','off',...
            'Position',[850 200 750 750] ,...
            'CloseRequestFcn',{@closeConnections,emgDataPort,auxDataPort,...
            legacyEmgDataPort,legacyAuxDataPort,commPort});
    end


%% OPEN COMMAND PORT AND CONFIGURE TCU
    function [samplesPerFrame, frameInterval] = openConnection()
        %Open Command port connection
        try
            fopen(commPort);
        catch
            closeConnections(figureHandleEMG,figureHandleAUX,0,emgDataPort,auxDataPort,...
                legacyEmgDataPort,legacyAuxDataPort,commPort);
            error(['CONNECTION ERROR: Please start the Delsys '...
                'Trigno Control Application and try again']);
            return
        end
        
        %Configure + check TCU status
        [samplesPerFrame, frameInterval] = setTrignoStatus(commPort);
    end
%% CONFIGURE DATA MANAGEMENT SOLUTION
    function [emgDataArray,emgBytesToRead,emgSampleCounter] = configureDataManager(samplesPerFrame, frameInterval)
        %Calculate bytes to read from each port to maintain synchronization
        %using the formula:
        %   (samples/frame) * (number of channels) * (4 bytes/sample)
        bytesToRead = samplesPerFrame .* numChannels * 4;
        
        %Preallocate data arrays (include some buffer to account for system
        %timing)
        numFrames = ceil(runTime / frameInterval) + 75;
        totalSamples = samplesPerFrame * numFrames;
        
        
        %Output data type
        dataType = 'single';
        %EMG Data Port
        emgDataArray = zeros(totalSamples(1),numChannels(1),dataType);
        emgDataArrayToSave = zeros(totalSamples(1),numChannels(1),dataType);
        emgBytesToRead = bytesToRead(1);
        emgSampleCounter = 0;
        emgSampleCounterToSave = 0;
        %         for itrial = 1:10
        %             data{itrial} = zeros(totalSamples(1),numChannels(1),dataType);
        %         end
        %AUX Data Port
        auxDataArray = zeros(totalSamples(2),numChannels(2),dataType);
        auxDataArrayToSave = zeros(totalSamples(2),numChannels(2),dataType);
        auxBytesToRead = bytesToRead(2);
        auxSampleCounter = 0;
        auxSampleCounterToSave = 0;
        
        %Legacy EMG Data Port
        legacyEmgDataArray = zeros(totalSamples(3),numChannels(3),dataType);
        legacyEmgBytesToRead = bytesToRead(3);
        legacyEmgSampleCounter = 0;
        
        %Legacy AUX Data Port
        legacyAuxDataArray = zeros(totalSamples(4),numChannels(4),dataType);
        legacyAuxBytesToRead = bytesToRead(4);
        legacyAuxSampleCounter = 0;
        
        %Implement callback functions for each port
        %   IMPORTANT: users should periodically clear the input buffers of
        %   all data ports to prevent the SDK from choking, even when not using
        %   said data.
        %
        %   NOTE: this implementation uses separate nested functions for each
        %   port in order to easily gain access to incoming data within the
        %   MATLAB workspace, at the expense of code redundancy. Other
        %   implementations should certainly be considered.
    end


%% CONFIGURE EMG DATA PORT
    function [emgPlotBuffer,downsampleRate] = configureDataport(emgBytesToRead,samplesPerFrame,frameInterval,fcnMode)
        
        emgDataPort.BytesAvailableFcn = {@readDataPortEMG};
        emgDataPort.BytesAvailableFcnMode = fcnMode;
        emgDataPort.BytesAvailableFcnCount = emgBytesToRead;
        
        %EMG plot buffer
        downsampleRate = 5;
        %Plot data buffer size
        plotBuffer = 5; %s
        emgPlotBuffer = ceil(plotBuffer * samplesPerFrame(1) / ...
            (frameInterval * downsampleRate));
        
        function readDataPortEMG(~,~)
            
            %Check that enough bytes are available to read
            bytesReady = emgDataPort.BytesAvailable;
            bytesReady = bytesReady - mod(bytesReady,emgBytesToRead);
            if bytesReady==0
                %Not enough bytes, return to main function
                
                return
            end
            %             disp('read data emg port')
            if (EMGStarted == 0)
                disp('start display')
                
                EMGStarted = 1;
            end
            
            %Enough bytes available, read data and reshape data array
            c = size(emgDataArray,2);
            newData = cast(fread(emgDataPort, bytesReady), 'uint8');
            newData = typecast(newData, dataType);
            r = length(newData) / c;
            newData = reshape(newData, c, r)';
            
            %Append data to existing data array and increment sample counter
            rowsToAppend = (1:r) + emgSampleCounter;
            emgDataArray(rowsToAppend,:) = newData;
            emgSampleCounter = emgSampleCounter + r;
            
            %create array to save data
            rowsToAppendToSave = (1:r) + emgSampleCounterToSave;
            emgDataArrayToSave(rowsToAppendToSave,:) = newData;
            emgSampleCounterToSave = emgSampleCounterToSave + r;
        end
    end
%% CONFIGURE AUX DATA PORT
    function [] = configureAuxDataport(fcnMode)
        
        auxDataPort.BytesAvailableFcn = {@readDataPortAUX};
        auxDataPort.BytesAvailableFcnMode = fcnMode;
        auxDataPort.BytesAvailableFcnCount = auxBytesToRead;
        %     auxDataPort.BytesAvailableFcnCount = 4;
        
        
        %aux plot buffer
        downsampleRate = 5;
        %Plot data buffer size
        plotBuffer = 5; %s
        auxPlotBuffer = ceil(plotBuffer * samplesPerFrame(2) / ...
            (frameInterval * downsampleRate));
        
        function readDataPortAUX(~,~)
            
            %Check that enough bytes are available to read
            bytesReady = auxDataPort.BytesAvailable;
            bytesReady = bytesReady - mod(bytesReady,auxBytesToRead);
            if bytesReady==0
                %Not enough bytes, return to main function
                return
            end
            
            %Enough bytes available, read data and reshape data array
            c = size(auxDataArray,2);
            newData = cast(fread(auxDataPort, bytesReady), 'uint8');
            newData = typecast(newData, dataType);
            r = length(newData) / c;
            newData = reshape(newData, c, r)';
            
            %Append data to existing data array and increment sample counter
            rowsToAppend = (1:r) + auxSampleCounter;
            auxDataArray(rowsToAppend,:) = newData;
            auxSampleCounter = auxSampleCounter + r;
            
            %Append data to existing data array and increment sample counter
            rowsToAppendToSave = (1:r) + auxSampleCounterToSave;
            auxDataArrayToSave(rowsToAppendToSave,:) = newData;
            auxSampleCounterToSave = auxSampleCounterToSave + r;
            
        end
    end
%% CONFIGURE LEGACY EMG DATA PORT
    function [] = configureLegacyDataport()
        
        legacyEmgDataPort.BytesAvailableFcn = {@readDataPortLegacyEMG};
        legacyEmgDataPort.BytesAvailableFcnMode = fcnMode;
        legacyEmgDataPort.BytesAvailableFcnCount = legacyEmgBytesToRead;
        
        function readDataPortLegacyEMG(~,~)
            
            %Check that enough bytes are available to read
            bytesReady = legacyEmgDataPort.BytesAvailable;
            bytesReady = bytesReady - mod(bytesReady,legacyEmgBytesToRead);
            if bytesReady==0
                %Not enough bytes, return to main function
                return
            end
            
            %DONT CARE JUST READ TO CLEAR BUFFER
            fread(legacyEmgDataPort, bytesReady);
            
            %         %Enough bytes available, read data and reshape data array
            %         c = size(legacyEmgDataArray,2);
            %         newData = cast(fread(legacyEmgDataPort, bytesReady), 'uint8');
            %         newData = typecast(newData, dataType);
            %         r = length(newData) / c;
            %         newData = reshape(newData, c, r)';
            %
            %         %Append data to existing data array and increment sample counter
            %         rowsToAppend = (1:r) + legacyEmgSampleCounter;
            %         legacyEmgDataArray(rowsToAppend,:) = newData;
            %         legacyEmgSampleCounter = legacyEmgSampleCounter + r;
            
        end
    end
%% CONFIGURE LEGACY AUX DATA PORT
    function [] = configureLegacyAuxDataport()
        
        legacyAuxDataPort.BytesAvailableFcn = {@readDataPortLegacyAUX};
        legacyAuxDataPort.BytesAvailableFcnMode = fcnMode;
        legacyAuxDataPort.BytesAvailableFcnCount = legacyAuxBytesToRead;
        
        function readDataPortLegacyAUX(~,~)
            
            %Check that enough bytes are available to read
            bytesReady = legacyAuxDataPort.BytesAvailable;
            bytesReady = bytesReady - mod(bytesReady,legacyAuxBytesToRead);
            if bytesReady==0
                %Not enough bytes, return to main function
                return
            end
            
            %DONT CARE JUST READ TO CLEAR BUFFER
            fread(legacyAuxDataPort, bytesReady);
            
            %         %Enough bytes available, read data and reshape data array
            %         c = size(legacyAuxDataArray,2);
            %         newData = cast(fread(legacyAuxDataPort, bytesReady), 'uint8');
            %         newData = typecast(newData, dataType);
            %         r = length(newData) / c;
            %         newData = reshape(newData, c, r)';
            %
            %         %Append data to existing data array and increment sample counter
            %         rowsToAppend = (1:r) + legacyAuxSampleCounter;
            %         legacyAuxDataArray(rowsToAppend,:) = newData;
            %         legacyAuxSampleCounter = legacyAuxSampleCounter + r;
            
        end
    end
%% BUILD PLOT AXES

% plotHandlesEMG = buildPlotAxes(figureHandleEMG,emgPlotBuffer);


%% UPDATE PLOT

    function updateEmgPlot(plotHandlesEMG,emgDataArray,emgSampleCounter,emgPlotBuffer,downsampleRate)
        
        %Check amount of existing data
        if emgSampleCounter >= (emgPlotBuffer*downsampleRate)
            %Use most recent data to fill plot buffer
            
            plotIdx = (1:downsampleRate:(emgPlotBuffer*downsampleRate)) + ...
                (emgSampleCounter - (emgPlotBuffer*downsampleRate) - ...
                rem(emgSampleCounter,downsampleRate));
        else
            %Not enough data to fill plot buffer
            
            plotIdx = 1:downsampleRate:(emgSampleCounter - ...
                rem(emgSampleCounter,downsampleRate));
        end
        
        %Update plots
        for ii = 1:size(plotHandlesEMG,1)
            plotHandlesEMG(ii).YData = emgDataArray(plotIdx,ii);
        end
        
        drawnow;
        
    end

%% UPDATE PLOT

    function updateAuxPlot(plotHandlesAUX,auxDataArray,auxSampleCounter, auxPlotBuffer,downsampleRate)
        
        %Check amount of existing data
        if auxSampleCounter >= (auxPlotBuffer*downsampleRate)
            %Use most recent data to fill plot buffer
            
            plotIdx = (1:downsampleRate:(auxPlotBuffer*downsampleRate)) + ...
                (auxSampleCounter - (auxPlotBuffer*downsampleRate) - ...
                rem(auxSampleCounter,downsampleRate));
        else
            %Not enough data to fill plot buffer
            
            plotIdx = 1:downsampleRate:(auxSampleCounter - ...
                rem(auxSampleCounter,downsampleRate));
        end
        
        %Update plots
        for ii = 1:9:size(plotHandlesAUX,1)
           
            plotHandlesAUX(ii).YData = auxDataArray(plotIdx,ii);
            plotHandlesAUX(ii+1).YData = auxDataArray(plotIdx,ii+1);
            plotHandlesAUX(ii+2).YData = auxDataArray(plotIdx,ii+2);

                                  
        end
        
        drawnow;
        
    end

%% START DATA STREAM
    function [] = dataStream()
        %Open data ports
        
        disp('open data ports');
        
        try
            fopen(emgDataPort);
            fopen(auxDataPort);
            
            
            %         fopen(legacyEmgDataPort);
            %         fopen(legacyAuxDataPort);
        catch
            closeConnections(figureHandleEMG,figureHandleAUX,0,emgDataPort,auxDataPort,...
                legacyEmgDataPort,legacyAuxDataPort,commPort);
            error(['CONNECTION ERROR: Please start the Delsys '...
                'Trigno Control Application and try again']);
        end
        
        
        
    end
%Extended pause to allow axes to render before streaming

% pause(5);
% %Start data streaming

% %start(t);
% %wait(t);
%
%
%

%Assign data arrays to outputs
% varargout{1} = emgDataArray;
% varargout{2} = auxDataArray;
% varargout{3} = legacyEmgDataArray;
% varargout{4} = legacyAuxDataArray;

% %Delete timer, delete figure, close connections
% closeConnections(figureHandleEMG,0,emgDataPort,auxDataPort,...
%     legacyEmgDataPort,legacyAuxDataPort,commPort);



end


%% LOCAL FUNCTION FOR CONFIGURING TRIGNO CONTROL UTILITY

function [samplesPerFrame,frameInterval] = setTrignoStatus(commPort)
%SETTRIGNOSTATUS configures and queries current state of the Trigno
%Control Utility and returns the samples per frame of each data port as
%a 4-element column vector:
%   [1] emgSamples = samples per frame of EMG Data port (50043)
%   [2] auxSamples = samples per frame of AUX Data port (50044)
%   [3] legacyEmgSamples = samples per frame of EMG Data port (50041)
%   [4] legacyAuxSamples = samples per frame of AUX Data port (50042)

pauseLength = 1.0; %seconds

%Clear Commmand port buffer
pause(pauseLength);
fread(commPort,commPort.BytesAvailable);

%% CONFIGURE TRIGNO CONTROL UTILITY

%Configure TCU
fprintf(commPort, sprintf('BACKWARDS COMPATIBILITY ON\r\n\r'));
pause(pauseLength);
fread(commPort,commPort.BytesAvailable);
fprintf(commPort, sprintf('UPSAMPLE ON\r\n\r'));
pause(pauseLength);
fread(commPort,commPort.BytesAvailable);

%%
%Configure Trigger
fprintf(commPort, sprintf('TRIGGER START ON\r\n\r'));
pause(pauseLength);
fread(commPort,commPort.BytesAvailable);

fprintf(commPort, sprintf('TRIGGER STOP OFF\r\n\r'));
pause(pauseLength);
fread(commPort,commPort.BytesAvailable);

%% QUERY TRIGNO CONTROL UTILITY STATE

%Query TCU backwards compatibility status
fprintf(commPort, sprintf('BACKWARDS COMPATIBILITY?\r\n\r'));
pause(pauseLength);
backCompatibilityFlag = fread(commPort,commPort.BytesAvailable);
backCompatibilityFlag = strtrim(char(backCompatibilityFlag'));

%Query TCU upsampling status
fprintf(commPort, sprintf('UPSAMPLING?\r\n\r'));
pause(pauseLength);
upsampleFlag = fread(commPort,commPort.BytesAvailable);
upsampleFlag = strtrim(char(upsampleFlag'));

%Query frame interval
%     fprintf(commPort, sprintf('FRAME INTERVAL?\r\n\r'));
%     pause(pauseLength);
%     frameBytes = fread(commPort,commPort.BytesAvailable);
%     frameInterval = str2double(strtrim(char(frameBytes')));
frameInterval = .0135;


%% CALCULATE SAMPLES PER FRAME

if strcmp(backCompatibilityFlag,'YES')
    %Backwards Compatibility ON
    if strcmp(upsampleFlag,'UPSAMPLING ON')
        %Upsampling ON
        emgSamples = 27;
        legacyEmgSamples = emgSamples;
    else
        %Upsampling OFF
        emgSamples = 26;
        legacyEmgSamples = 15;
    end
    auxSamples = 2;
    legacyAuxSamples = auxSamples;
else
    %Backwards Compatibility OFF
    %Query EMG data port samples per frame
    fprintf(commPort, sprintf('MAX SAMPLES EMG?\r\n\r'));
    pause(pauseLength);
    emgSamples = fread(commPort,commPort.BytesAvailable);
    
    %Query AUX data port samples per frame
    fprintf(commPort, sprintf('MAX SAMPLES AUX?\r\n\r'));
    pause(pauseLength);
    auxSamples = fread(commPort,commPort.BytesAvailable);
    
    %Legacy data ports upsampled to same max sampling rates
    legacyEmgSamples = emgSamples;
    legacyAuxSamples = auxSamples;
end

samplesPerFrame = [emgSamples;...
    auxSamples; legacyEmgSamples; legacyAuxSamples];

end

%% LOCAL TIMER START FUNCTION

function startAcquisition(commPort)

%Start data acquisition
fprintf(commPort, sprintf('START\r\n\r'));
disp('START');

end


%% LOCAL TIMER STOP FUNCTION

function stopAcquisition(commPort)

%Stop data acquisition after allotted run time
fprintf(commPort, sprintf('STOP\r\n\r'));
disp('STOP');

end

%% LOCAL FUNCTION FOR BUILDING PLOT AXES

function plotHandles = buildPlotAxes(figureHandle,emgPlotBuffer)

%Make figure active
figure(figureHandle);

%EMG axis + plot handle preallocation
numChannels = 16;
axesHandles = gobjects(numChannels,1);
plotHandles = gobjects(numChannels,1);

for ii = 1:numChannels
    
    %Create axis
    axesHandles(ii) = subplot(4,4,ii);
    %         axesHandles(ii) = subplot(1,1,1);
    
    %Set trace parameters
    plotHandles(ii) = plot(axesHandles(ii),0,'-',...
        'LineWidth',1,...
        'Color',[0 1 1]);
    
    %Set axis parameters
    axesHandles(ii).YGrid = 'on';
    axesHandles(ii).XGrid = 'on';
    axesHandles(ii).Color = [.15 .15 .15];
    axesHandles(ii).YLim = [-.003 .003];
    axesHandles(ii).YLimMode = 'manual';
    axesHandles(ii).XLim = [1 emgPlotBuffer];
    axesHandles(ii).XLimMode = 'manual';
    ylabel(axesHandles(ii),'V');
    xlabel(axesHandles(ii),'Samples');
    
    if mod(ii,4) == 1
        ylabel(axesHandles(ii),'V');
    else
        axesHandles(ii).YTickLabel = '';
        ylabel(axesHandles(ii),'');
    end
    
    if ii > 12
        xlabel(axesHandles(ii),'Samples');
    else
        axesHandles(ii).XTickLabel = '';
        xlabel(axesHandles(ii),'');
    end
    
    title(sprintf('EMG Ch. %i', ii));
    
end

%     %To enable linked zoom
%     linkaxes(axesHandles);

end

%% LOCAL FUNCTION FOR BUILDING PLOT AXES

function plotHandles = buildPlotAuxAxes(figureHandle,emgPlotBuffer)

%Make figure active
figure(figureHandle);

%EMG axis + plot handle preallocation
numChannels = 16;
% numChannels = 1;

axesHandles = gobjects(numChannels,1);
plotHandles = gobjects(numChannels*9,1);

for ii = 1:numChannels
    
    %Create axis
    axesHandles(ii) = subplot(4,4,ii);
    %         axesHandles(ii) = subplot(1,1,1);
    
    %Set trace parameters
    hold on
    plotHandles(ii*9-8) = plot(axesHandles(ii),0,'-',...
        'LineWidth',1,...
        'Color',[1 0 0]);
    plotHandles(ii*9-7) = plot(axesHandles(ii),0,'-',...
        'LineWidth',1,...
        'Color',[0 1 0]);
    plotHandles(ii*9-6) = plot(axesHandles(ii),0,'-',...
        'LineWidth',1,...
        'Color',[0 0 1]);
    hold off
    
    %Set axis parameters
    axesHandles(ii).YGrid = 'on';
    axesHandles(ii).XGrid = 'on';
    axesHandles(ii).Color = [.15 .15 .15];
%     axesHandles(ii).YLim = [-.003 .003];
    axesHandles(ii).YLimMode = 'manual';
    axesHandles(ii).XLim = [1 emgPlotBuffer];
    axesHandles(ii).XLimMode = 'manual';
    ylabel(axesHandles(ii),'V');
    xlabel(axesHandles(ii),'Samples');
    
    if mod(ii,4) == 1
        ylabel(axesHandles(ii),'m/s^2');
    else
        axesHandles(ii).YTickLabel = '';
        ylabel(axesHandles(ii),'');
    end
    
    if ii > 12
        xlabel(axesHandles(ii),'Samples');
    else
        axesHandles(ii).XTickLabel = '';
        xlabel(axesHandles(ii),'');
    end
    
    title(sprintf('EMG Ch. %i', ii));
    
end

%     %To enable linked zoom
%     linkaxes(axesHandles);

end
%% LOCAL FUNCTION FOR CLOSING ALL TIMERS + TCP/IP OBJECTS

function closeConnections(figureHandle,figureHandle2,~,...
    emgPort,auxPort,legacyEmgPort,legacyAuxPort,commPort)


%Close all open ports
if isvalid(emgPort)
    fclose(emgPort);
%         delete(emgPort);
%         clear emgPort;
end
if isvalid(auxPort)
    fclose(auxPort);
%     delete(auxPort);
%     clear auxPort;
end
% if isvalid(legacyEmgPort)
%     fclose(legacyEmgPort);
%     delete(legacyEmgPort);
%     clear legacyEmgPort;
% end
% if isvalid(legacyAuxPort)
%     fclose(legacyAuxPort);
%     delete(legacyAuxPort);
%     clear legacyAuxPort;
% end
if isvalid(commPort)
    fclose(commPort);
%         delete(commPort);
%         clear commPort;
    
end

%Close figure window
delete(figureHandle);
delete(figureHandle2);

end


% %% LOCAL FUNCTION FOR CLOSING ALL TIMERS + TCP/IP OBJECTS
%
% function closeConnections(figureHandle,~,timerObject,...
%     emgPort,auxPort,legacyEmgPort,legacyAuxPort,commPort)
%
% %Stop + delete timer
% if isvalid(timerObject)
%     stop(timerObject);
%     delete(timerObject);
% end
%
% %Close all open ports
% if isvalid(emgPort)
%     fclose(emgPort);
%     delete(emgPort);
%     clear emgPort;
% end
% if isvalid(auxPort)
%     fclose(auxPort);
%     delete(auxPort);
%     clear auxPort;
% end
% if isvalid(legacyEmgPort)
%     fclose(legacyEmgPort);
%     delete(legacyEmgPort);
%     clear legacyEmgPort;
% end
% if isvalid(legacyAuxPort)
%     fclose(legacyAuxPort);
%     delete(legacyAuxPort);
%     clear legacyAuxPort;
% end
% if isvalid(commPort)
%     fclose(commPort);
%     delete(commPort);
%     clear commPort;
% end
%
% %Close figure window
% delete(figureHandle);
%
% end
%
