classdef lpms < handle
    % Lpms class to interface with LpmsSensors
    %
    % Known Issues:
    % - Serial Interrupt routine blocks main processing thread
    %   when transferring at data rate > 100Hz 
    %
    % TODO: 
    % - Implement 16bit data parsing
    %
    %
    % Author: H.E.Yap
    % Date: 2016/07/19    
    % Revision: 0.1 
    % Copyright: LP-Research Inc. 2016
    
    properties (Constant)
        PACKET_ADDRESS0     = 0;
        PACKET_ADDRESS1     = 1;
        PACKET_FUNCTION0    = 2;
        PACKET_FUNCTION1    = 3;
        PACKET_LENGTH0      = 4;
        PACKET_LENGTH1      = 5;
        PACKET_RAW_DATA     = 6;
        PACKET_LRC_CHECK0   = 7;
        PACKET_LRC_CHECK1   = 8;
        PACKET_END          = 9;
        MAX_BUFFER = 4096;
        
        % Command register
        REPLY_ACK             = 0;
        REPLY_NACK            = 1;
        GET_CONFIG            = 4;
        GET_STATUS            = 5;
        GOTO_COMMAND_MODE     = 6;
        GOTO_STREAM_MODE      = 7;
        GET_SENSOR_DATA       = 9;
        
        GET_SERIAL_NUMBER     = 90;
        GET_DEVICE_NAME       = 91;
        GET_FIRMWARE_INFO     = 92;
        
        %Configuration register contents
        LPMS_GYR_AUTOCAL_ENABLED = bitshift(1, 30);
        LPMS_LPBUS_DATA_MODE_16BIT_ENABLED = bitshift(1, 22);
        LPMS_LINACC_OUTPUT_ENABLED = bitshift(1, 21);
        LPMS_DYNAMIC_COVAR_ENABLED = bitshift(1, 20);
        LPMS_ALTITUDE_OUTPUT_ENABLED = bitshift(1, 19);
        LPMS_QUAT_OUTPUT_ENABLED = bitshift(1, 18);
        LPMS_EULER_OUTPUT_ENABLED = bitshift(1, 17);
        LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED = bitshift(1, 16);
        LPMS_GYR_CALIBRA_ENABLED = bitshift(1, 15);
        LPMS_HEAVEMOTION_OUTPUT_ENABLED = bitshift(1, 14);
        LPMS_TEMPERATURE_OUTPUT_ENABLED = bitshift(1, 13);
        LPMS_GYR_RAW_OUTPUT_ENABLED = bitshift(1, 12);
        LPMS_ACC_RAW_OUTPUT_ENABLED = bitshift(1, 11);
        LPMS_MAG_RAW_OUTPUT_ENABLED = bitshift(1, 10);
        LPMS_PRESSURE_OUTPUT_ENABLED = bitshift(1, 9);
        LPMS_STREAM_FREQ_5HZ_ENABLED      = 0;
        LPMS_STREAM_FREQ_10HZ_ENABLED     = 1;
        LPMS_STREAM_FREQ_25HZ_ENABLED     = 2;
        LPMS_STREAM_FREQ_50HZ_ENABLED     = 3;
        LPMS_STREAM_FREQ_100HZ_ENABLED    = 4;
        LPMS_STREAM_FREQ_200HZ_ENABLED    = 5;
        LPMS_STREAM_FREQ_400HZ_ENABLED    = 6;
        LPMS_STREAM_FREQ_MASK             = 7;
        
        LPMS_STREAM_FREQ_5HZ = 5;
        LPMS_STREAM_FREQ_10HZ = 10;
        LPMS_STREAM_FREQ_25HZ = 25;
        LPMS_STREAM_FREQ_50HZ = 50;
        LPMS_STREAM_FREQ_100HZ = 100;
        LPMS_STREAM_FREQ_200HZ = 200;
        LPMS_STREAM_FREQ_400HZ = 400;
        
        PARAMETER_SET_DELAY = 0.01;
        DATA_QUEUE_SIZE = 64;
    end
   
    properties
        % serial
        serConn;
        isSensorConnected = false;
        
        % define the properties of the class here, (like fields of a struct)
        rxBuffer = uint8(zeros(1, lpms.MAX_BUFFER));
        rawTxBuffer = uint8(zeros(1, lpms.MAX_BUFFER));
        inBytes = uint8(zeros(1, 2));
        rxState = lpms.PACKET_END;
        rxIndex = 0;
        currentAddress = 0;
        currentFunction = 0;
        currentLength = 0;
        lrcCheck = 0;
        lastTimestamp = 0;
        fps = 0;
        %
        waitForAck = false;
        waitForData = false;
        % Settings related
        imuId = 0;
        gyrRange = 0;
        accRange = 0;
        magRange = 0;
        streamingFrequency = 0;
        filterMode = 0;
        isStreamMode = true;

        configurationRegister = 0;
        configurationRegisterReady = false;
        sensorDataLength = 0;
        serialNumber = 'none';
        serialNumberReady = false;
        deviceName = 'none';
        deviceNameReady = false;
        firmwareInfo= 'none';
        firmwareInfoReady = false;
        firmwareVersion = 'none';

        accEnable = false;
        gyrEnable = false;
        magEnable = false;
        angularVelEnable = false;
        quaternionEnable = false;
        eulerAngleEnable = false;
        linAccEnable = false;
        pressureEnable = false;
        altitudeEnable = false;
        temperatureEnable = false;
        heaveEnable = false;
        sixteenBitDataEnable = false;
        resetTimestampFlag = false;
        
        % sensorData
        sensorData = struct(...
            'timestamp', 0.0, ...
            'gyr',  zeros(1,3), ...
            'acc',  zeros(1,3), ...
            'mag',  zeros(1,3), ...
            'angVel',  zeros(1,3), ...
            'quat',  zeros(1,4), ...
            'euler',  zeros(1,3), ...
            'linAcc',  zeros(1,3), ...
            'pressure', 0.0, ...
            'altitude', 0.0, ...
            'temperature', 0.0, ...
            'heave', 0.0 ...
        );
    
        dataQueue = [];
    end
    
    methods
        function ret = connect(obj, serPort, baudrate)
            oldSerial = instrfind('Port', serPort);
            if (~isempty(oldSerial))
                delete(oldSerial);
            end
            obj.serConn = serial(serPort, 'TimeOut', 1, 'BaudRate', baudrate);
            
            % Force put sensor in command mode
            try
                fopen(obj.serConn);
                obj.lpBusSetDataNone(obj.GOTO_COMMAND_MODE);
                fclose(obj.serConn);
            catch e
                errordlg(e.message);
                obj.isSensorConnected = false;
            end
            
            % Get sensor configuration
            obj.serConn.BytesAvailableFcnCount = 1;
            obj.serConn.BytesAvailableFcnMode = 'byte';
            obj.serConn.BytesAvailableFcn = @obj.bytesAvailable_Callback; 
            try
                fopen(obj.serConn);
                obj.isSensorConnected = true;
                obj.setCommandMode();
                obj.getConfig();
            catch e
                errordlg(e.message);
                obj.isSensorConnected = false;
            end
            
            fclose(obj.serConn);
            
            obj.serConn.BytesAvailableFcnCount = 111; 
            obj.serConn.BytesAvailableFcnMode = 'byte';
            obj.serConn.BytesAvailableFcn = @obj.bytesAvailable_Callback; 
            try
                fopen(obj.serConn);
                obj.isSensorConnected = true;
            catch e
                errordlg(e.message);
                obj.isSensorConnected = false;
            end
            
            tic
            ret = obj.isSensorConnected;
        end
        
        function ret = disconnect(obj)
            fclose(obj.serConn);
            obj.isSensorConnected = false;
            ret = true;
        end
        
        function ret = isConnected(obj)
            ret = obj.isSensorConnected;
        end
        
        function obj = setCommandMode(obj)
            obj.waitForAck = true;
            obj.lpBusSetDataNone(obj.GOTO_COMMAND_MODE);
            if ~obj.waitForAckLoop()
                % timeout, manual read bytes avaiable
                % disp('setCommandMode wait for ack timeout')
                obj.bytesAvailable_Callback(obj.serConn);
            end
            obj.isStreamMode = false;
        end
        
        function obj = setStreamingMode(obj)
            obj.waitForAck = true;
            obj.lpBusSetDataNone(obj.GOTO_STREAM_MODE);
            obj.waitForAckLoop();
            obj.isStreamMode = true;
        end
        
        function ret = getConfig(obj)
            obj.waitForData = true;
            obj.lpBusSetDataNone(obj.GET_CONFIG);
            if ~obj.waitForDataLoop()
                % timeout, manual read bytes avaiable
                % disp('getConfig wait for data timeout')
                obj.bytesAvailable_Callback(obj.serConn);
            end
            timeout = 0;
            while obj.configurationRegister == 0 && timeout < 100
                pause(obj.PARAMETER_SET_DELAY)
                timeout = timeout + 1;
            end
            if timeout >= 100
                ret = false;
            else
                ret = true;
            end
        end
        
        function ret = getCurrentSensorData(obj)
            % Retrieve newest data from sensor 
            if (obj.isStreamMode)
                ret = obj.sensorData;
            else
                obj.waitForData = true;
                obj.lpBusSetDataNone(obj.GET_SENSOR_DATA);
                obj.waitForDataLoop();
                ret = obj.sensorData;
            end
        end
        
        function ret = getQueueSensorData(obj)
            % Retrieve oldest data in data queue
            if ~isempty(obj.dataQueue)
                ret = obj.dataQueue(1);
                obj.dataQueue = obj.dataQueue(2:end);
            else
                ret = [];
            end
        end
        
        function ret = hasSensorData(obj)
            ret = length(obj.dataQueue);
        end
        
    end
    
    
    methods (Access = private)
        function bytesAvailable_Callback(obj, hObject, eventdata)
            if (obj.isSensorConnected)
                try 
                    n = hObject.BytesAvailable;
                    if n > 0 
                        [RxText, nBytes] = fread(hObject,n);
                        obj.parse(RxText, nBytes);
                    end
                catch e
                    disp(e)
                    disp(e.message)
                end
            end
        end

        function obj = parse(obj, buf, nBytes)
            for i=1:nBytes
                d = buf(i);
                switch obj.rxState 
                   case obj.PACKET_END
                       if (d == uint8(':'))
                           obj.rxState = obj.PACKET_ADDRESS0;
                       end

                   case obj.PACKET_ADDRESS0
                       obj.inBytes(1) = d;
                       obj.rxState = obj.PACKET_ADDRESS1;

                   case obj.PACKET_ADDRESS1
                       obj.inBytes(2) = d;
                       obj.currentAddress = typecast(obj.inBytes,'uint16');
                       obj.imuId = obj.currentAddress;
                       obj.rxState = obj.PACKET_FUNCTION0;

                   case obj.PACKET_FUNCTION0
                       obj.inBytes(1) = d;
                       obj.rxState = obj.PACKET_FUNCTION1;

                   case obj.PACKET_FUNCTION1
                       obj.inBytes(2) = d;
                       obj.currentFunction = typecast(obj.inBytes,'uint16');
                       obj.rxState = obj.PACKET_LENGTH0;

                   case obj.PACKET_LENGTH0
                       obj.inBytes(1) = d;
                       obj.rxState = obj.PACKET_LENGTH1;

                   case obj.PACKET_LENGTH1
                       obj.inBytes(2) = d;
                       obj.currentLength = typecast(obj.inBytes,'uint16');
                       obj.rxIndex = 0;
                       obj.rxState = obj.PACKET_RAW_DATA;
                       obj.lrcCheck = obj.currentAddress + obj.currentFunction + obj.currentLength;
                      

                   case obj.PACKET_RAW_DATA
                       if obj.rxIndex == obj.currentLength
                           obj.inBytes(1) = d;
                           obj.rxState = obj.PACKET_LRC_CHECK1; 
                       else
                           if (obj.rxIndex < obj.MAX_BUFFER)
                               obj.rxBuffer(obj.rxIndex+1) = d;
                               obj.rxIndex = obj.rxIndex + 1;
                               obj.lrcCheck = obj.lrcCheck + d;
                           else
                               obj.rxState = obj.PACKET_END;
                           end
                       end

                   case obj.PACKET_LRC_CHECK1
                       obj.inBytes(2) = d;
                       lrcReceived = typecast(obj.inBytes,'uint16');
                       if (lrcReceived == obj.lrcCheck)
                           obj.parseFunction();  
                       end
                       obj.rxState = obj.PACKET_END;
                      
                   otherwise 
                       obj.rxState = obj.PACKET_END;
                        
                      
                end
            end
        end
        
        function ret = waitForAckLoop(This)
            timeout = 0;
            while (This.waitForAck && timeout < 100)
                pause(This.PARAMETER_SET_DELAY);
                timeout = timeout + 1;
            end
            if timeout >= 100
                ret = false;
            else
                ret = true;
            end
        end
        
        function ret = waitForDataLoop(This)
            timeout = 0;
            while (This.waitForData && timeout < 100)
                pause(This.PARAMETER_SET_DELAY);
                timeout = timeout + 1;
            end
            if timeout >= 100
                ret = false;
            else
                ret = true;
            end
        end
        
        function lpBusSetDataNone(This, command)
            This.sendData(command, 0);
        end
        
        function sendData(This, command, length)
            txBuffer = zeros(1, 11+length);
            txBuffer(1) = hex2dec('3A');
            txBuffer(2:3) = typecast(uint16(This.imuId), 'uint8');
            txBuffer(4:5) = typecast(uint16(command), 'uint8');
            txBuffer(6:7) = typecast(uint16(length), 'uint8');
            txLrcCheck = 0;
            for i=2:7
                txLrcCheck = txLrcCheck + txBuffer(i);
            end
            for i = 1:length
                txBuffer(7+i) = This.rawTxData(i);
                txLrcCheck = txLrcCheck + This.rawTxData(i);
            end
            txBuffer(8+length:9+length) = typecast(uint16(txLrcCheck), 'uint8');
            txBuffer(10+length)=hex2dec('D');
            txBuffer(11+length)=hex2dec('A');
            for i=1:11+length
                fwrite(This.serConn,  char(txBuffer(i)));
            end
        end
        
        function parseFunction(This)
%             disp('Parse function')
            switch (This.currentFunction)
                case This.REPLY_ACK
%                     disp('REPLY_ACK') 
                    This.waitForAck = false;
                    
                case This.REPLY_NACK
%                     disp('REPLY_NACK') 
                    This.waitForAck = false;
            
                case This.GET_CONFIG
%                     disp('GET_CONFIG') 
                    This.configurationRegister = This.convertRxbytesToInt(0, This.rxBuffer);
                    This.parseConfig(This.configurationRegister);
                    This.waitForData = false;
                    
                case This.GET_SENSOR_DATA
                    %disp('GET_SENSOR_DATA')
                    t = toc;
                    This.fps = 1/t;
                    This.fps;
                    tic
                    This.parseSensorData();
                    This.waitForData = false;
                    
                case This.GET_DEVICE_NAME
%                     disp('GET_DEVICE_NAME')
                    This.deviceName = This.convertRxbytesToString(16, This.rxBuffer);
                    %deviceNameReady = true;
                    This.waitForData = false;
            end
        end
        
        function ret = convertRxbytesToInt(This, offset, buf)
             ret = typecast(buf(1+offset:4+offset),'uint32');
        end
                
        function ret = convertRxbytesToFloat(This, offset, buf)
             ret = typecast(buf(1+offset:4+offset),'single');
        end
        
        function ret = convertRxbytesToString(This, offset, buf)
             buf(1:offset)
             ret = char(buf(1:offset));
        end
        
        function ret = parseConfig(This, config)
             freqSettings = bitand(config, This.LPMS_STREAM_FREQ_MASK);
             if freqSettings == This.LPMS_STREAM_FREQ_5HZ_ENABLED
                 This.streamingFrequency = This.LPMS_STREAM_FREQ_5HZ;
             elseif freqSettings == This.LPMS_STREAM_FREQ_10HZ_ENABLED
                 This.streamingFrequency = This.LPMS_STREAM_FREQ_10HZ;
             elseif freqSettings == This.LPMS_STREAM_FREQ_25HZ_ENABLED
                 This.streamingFrequency = This.LPMS_STREAM_FREQ_25HZ;
             elseif freqSettings == This.LPMS_STREAM_FREQ_50HZ_ENABLED
                 This.streamingFrequency = This.LPMS_STREAM_FREQ_50HZ;
             elseif freqSettings == This.LPMS_STREAM_FREQ_100HZ_ENABLED
                 This.streamingFrequency = This.LPMS_STREAM_FREQ_100HZ;
             elseif freqSettings == This.LPMS_STREAM_FREQ_200HZ_ENABLED
                 This.streamingFrequency = This.LPMS_STREAM_FREQ_200HZ;
             elseif freqSettings == This.LPMS_STREAM_FREQ_400HZ_ENABLED
                 This.streamingFrequency = This.LPMS_STREAM_FREQ_400HZ;
             end
             
             if bitand(config, This.LPMS_LPBUS_DATA_MODE_16BIT_ENABLED)
                 This.sixteenBitDataEnable = true;
             else
                 This.sixteenBitDataEnable = false;
             end
             
             This.sensorDataLength=0;
             if bitand(config, This.LPMS_GYR_RAW_OUTPUT_ENABLED)
                 This.gyrEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 6;
                 else
                    This.sensorDataLength = This.sensorDataLength + 12;
                 end
             else
                 This.gyrEnable = false;
             end
             
             if bitand(config, This.LPMS_ACC_RAW_OUTPUT_ENABLED)
                 This.accEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 6;
                 else
                    This.sensorDataLength = This.sensorDataLength + 12;
                 end
             else
                 This.accEnable = false;
             end
             
             if bitand(config, This.LPMS_MAG_RAW_OUTPUT_ENABLED)
                 This.magEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 6;
                 else
                    This.sensorDataLength = This.sensorDataLength + 12;
                 end
             else
                 This.magEnable = false;
             end
             
             if bitand(config, This.LPMS_ANGULAR_VELOCITY_OUTPUT_ENABLED)
                 This.angularVelEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 6;
                 else
                    This.sensorDataLength = This.sensorDataLength + 12;
                 end
             else
                 This.angularVelEnable = false;
             end
             
             if bitand(config, This.LPMS_QUAT_OUTPUT_ENABLED)
                 This.quaternionEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 8;
                 else
                    This.sensorDataLength = This.sensorDataLength + 16;
                 end
             else
                 This.quaternionEnable = false;
             end
             
             if bitand(config, This.LPMS_EULER_OUTPUT_ENABLED)
                 This.eulerAngleEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 6;
                 else
                    This.sensorDataLength = This.sensorDataLength + 12;
                 end
             else
                 This.eulerAngleEnable = false;
             end
             
             if bitand(config, This.LPMS_LINACC_OUTPUT_ENABLED)
                 This.linAccEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 6;
                 else
                    This.sensorDataLength = This.sensorDataLength + 12;
                 end
             else
                 This.linAccEnable = false;
             end
             
             if bitand(config, This.LPMS_PRESSURE_OUTPUT_ENABLED)
                 This.pressureEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 2;
                 else
                    This.sensorDataLength = This.sensorDataLength + 4;
                 end
             else
                 This.pressureEnable = false;
             end
             
             if bitand(config, This.LPMS_TEMPERATURE_OUTPUT_ENABLED)
                 This.temperatureEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 2;
                 else
                    This.sensorDataLength = This.sensorDataLength + 4;
                 end
             else
                 This.temperatureEnable = false;
             end
             
             if bitand(config, This.LPMS_ALTITUDE_OUTPUT_ENABLED)
                 This.altitudeEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 2;
                 else
                    This.sensorDataLength = This.sensorDataLength + 4;
                 end
             else
                 This.altitudeEnable = false;
             end
             
             if bitand(config, This.LPMS_HEAVEMOTION_OUTPUT_ENABLED)
                 This.heaveEnable = true;
                 if (This.sixteenBitDataEnable)
                    This.sensorDataLength = This.sensorDataLength + 2;
                 else
                    This.sensorDataLength = This.sensorDataLength + 4;
                 end
             else
                 This.heaveEnable = false;
             end
             
        end
        
        function parseSensorData(This)
            % TODO: Implement 16bit data parsing
            r2d = 57.2958;
            This.sensorData.timestamp = double(This.convertRxbytesToInt(0, This.rxBuffer))*0.0025;
            if (~This.sixteenBitDataEnable)
                d = typecast(This.rxBuffer(5:This.sensorDataLength+4), 'single');
                o = 1;
                if This.gyrEnable
                    for i=1:3
                        This.sensorData.gyr(i) = d(o) * r2d;
                        o = o+1;
                    end
                end
                
                if This.accEnable
                    for i=1:3
                        This.sensorData.acc(i) = d(o);
                        o = o+1;
                    end
                end
                
                if This.magEnable
                    for i=1:3
                        This.sensorData.mag(i) = d(o);
                        o = o+1;
                    end
                end
                
                if This.angularVelEnable
                    for i=1:3
                       This.sensorData.angVel(i) = d(o)* r2d;
                       o = o+1;
                    end
                end
                
                if This.quaternionEnable
                    for i=1:4
                       This.sensorData.quat(i) = d(o);
                       o = o+1;
                    end
                end
                
                if This.eulerAngleEnable
                    for i=1:3
                       This.sensorData.euler(i) = d(o)* r2d;
                       o = o+1;
                    end
                end
                
                if This.linAccEnable
                    for i=1:3
                       This.sensorData.linAcc(i) = d(o);
                       o = o+1;
                    end
                end
                
                if This.pressureEnable
                    This.sensorData.pressure = d(o);
                    o = o+1;
                end
                
                if This.altitudeEnable
                    This.sensorData.altitude = d(o);
                    o = o+1;
                end
                
                if This.temperatureEnable
                    This.sensorData.temperature = d(o);
                    o = o+1;
                end
                
                if This.heaveEnable
                    This.sensorData.heave = d(o);
                end
                
            end
             
             % add data to queue
             if length(This.dataQueue) == This.DATA_QUEUE_SIZE
                 This.dataQueue = This.dataQueue(2:end);
             end
             This.dataQueue = [This.dataQueue This.sensorData];
             
             
        end
    end
end