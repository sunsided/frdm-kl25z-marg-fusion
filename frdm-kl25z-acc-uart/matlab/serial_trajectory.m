function serial_trajectory
    % SERIAL_TEST Reads streamed sensor data.
    %   define  
    %   global sensorDataCount accelBuffer gyroBuffer compassBuffer temperatureBuffer
    %   at the workspace after running this function to get the data.

    close all; clear all;

    % We turn it back on in the end
    cleanupHandler = onCleanup(@cleanUp);

    % Cleanup
    connectedPorts = instrfind();
    if ~isempty(connectedPorts)
        disp('Freeing connected ports.');
        for port=connectedPorts
            fclose(connectedPorts);
            delete(connectedPorts);
        end
    end
    
    % Configuring port object
    disp('Preparing serial port ...');
    global s;
    s = serial('COM3', ...
        'FlowControl', 'none', ...
        'BaudRate', 115200, ...
        'DataBits', 8, ...
        'Parity', 'none', ...
        'StopBits', 1, ...
        'TimeOut', 1, ...
        'InputBufferSize', 1024, ...
        'ReadAsyncMode', 'continuous', ...
        'Terminator', 0, ...
        'BytesAvailableFcnCount', 12, ...
        'BytesAvailableFcnMode', 'byte' ...
        );
    
    % Connecting to the port
    disp('Connecting to serial port ...');
    fopen(s);
    disp('Connected to serial port.');   
    
    % Grab jobject for fread
    sjobject = igetfield(s, 'jobject');
    
    % add data processing to path
    path(fullfile(fileparts(which(mfilename)), '..', '..', 'sensor-fusion', 'processing', 'set-1', 'trajectory'), path);
        
    % Prepare the plot
    preparePlotTrajectory();
    
    % prepare calibration
    loadCalibrationData();
    
    % Definitions
    global dataReady data
    prepareProtocolDecode();
    
    % Data counters
    global sensorDataCount ACCELEROMETER GYROSCOPE COMPASS TEMPERATURE;
    ACCELEROMETER = 1;
    GYROSCOPE     = 2;
    COMPASS       = 3;
    TEMPERATURE   = 4;
    sensorDataCount = zeros(4,1);
    totalDataCount = 0;
    
    % data buffers
    BUFFER_STEP_SIZE = 1000;
    global accelBuffer gyroBuffer compassBuffer temperatureBuffer
    accelBuffer         = zeros(BUFFER_STEP_SIZE, 4); % time + xyz
    gyroBuffer          = zeros(BUFFER_STEP_SIZE, 4); % time + xyz
    compassBuffer       = zeros(BUFFER_STEP_SIZE, 4); % time + xyz
    temperatureBuffer   = zeros(BUFFER_STEP_SIZE, 2); % time + t
        
    % Start timing for the graphics and data loop
    graphicsTimer = tic;
    dataTimer = tic;
    
    % Reading the data
    bulkSize = 80;
    while true
        % Read bytes; A profiler run showed that fread(s, count) is
        % horribly slow mainly due to error string formattings even
        % if they are not required. Because of this, a more low-level
        % variant of the function is called.
        out = fread(sjobject, bulkSize, 0, 0); % 0, 0 meaning unsigned int 8
        bytes = typecast(out(1), 'uint8');     % unfortunately it is not unsigned
        
        byteIndices = 1:numel(bytes);
        for b=byteIndices
            byte = bytes(b);
            
            % Attach byte to circular buffer
            %byteCircBuf = [byteCircBuf(2:end), double(byte)];

            % Decode the protocol
            protocolDecode(byte);

            % Decode data buffer
            if dataReady
                % Thanks, got it.
                dataReady = false;
                totalDataCount = totalDataCount + 1;
                timestamp = toc(dataTimer);
                
                % Attach NaN byte to circular buffer to aid debugging
                %byteCircBuf = [byteCircBuf(2:end), NaN];
                
                % Skip everything that is not from the MMA8451Q
                type = data(1);
                if type == 1
                    disp('MMA8451Q sensing disabled');
                    continue;
                    
                    % Decode MMA8451Q data
                    sensorDataCount(ACCELEROMETER) = sensorDataCount(ACCELEROMETER) + 1;
                    scaling = 4096;
                    
                    accXYZ = [
                        double(typecast(data(2:3), 'int16'));
                        double(typecast(data(4:5), 'int16'));
                        double(typecast(data(6:7), 'int16'));
                        ] / scaling;
                    continue;
                elseif type == 2
                    % Decode MPU6050 data
                    sensorDataCount(ACCELEROMETER) = sensorDataCount(ACCELEROMETER) + 1;
                    sensorDataCount(GYROSCOPE)     = sensorDataCount(GYROSCOPE) + 1;
                    sensorDataCount(TEMPERATURE)   = sensorDataCount(TEMPERATURE) + 1;
                    accScaling  = 8192; %16384 for 2g mode
                    gyroScaling = 131; %131 in 250°/s mode
                    tempScaling = 340;
                    tempoffset  = 36.53;
                     
                    % Swapping components due to orientation on my board
                    accXYZ = [
                        -double(typecast(data(4:5), 'int16'));
                         double(typecast(data(2:3), 'int16'));
                         double(typecast(data(6:7), 'int16'));
                        ] / accScaling;
                    gyroXYZ = [
                        -double(typecast(data(10:11), 'int16'));
                         double(typecast(data(8:9), 'int16'));
                         double(typecast(data(12:13), 'int16'));
                        ] / gyroScaling;
                    temperature = [
                        double(typecast(data(14:15), 'int16'));
                        ] / tempScaling + tempoffset;
                    
                    % attach data to buffer
                    if mod(sensorDataCount(ACCELEROMETER), BUFFER_STEP_SIZE) == 0
                        accelBuffer = [accelBuffer; zeros(BUFFER_STEP_SIZE, 4)];
                    end
                    accelBuffer(sensorDataCount(ACCELEROMETER), 1:4) = [timestamp; accXYZ];
                    
                    % attach data to buffer
                    if mod(sensorDataCount(GYROSCOPE), BUFFER_STEP_SIZE) == 0
                        gyroBuffer = [gyroBuffer; zeros(BUFFER_STEP_SIZE, 4)];
                    end
                    gyroBuffer(sensorDataCount(GYROSCOPE), 1:4) = [timestamp; gyroXYZ];
                    
                    % attach data to buffer
                    if mod(sensorDataCount(TEMPERATURE), BUFFER_STEP_SIZE) == 0
                        temperatureBuffer = [temperatureBuffer; zeros(BUFFER_STEP_SIZE, 2)];
                    end
                    temperatureBuffer(sensorDataCount(TEMPERATURE), 1:2) = [timestamp; temperature];
                    
                elseif type == 3
                    % Decode HMC5883L data
                    sensorDataCount(COMPASS)        = sensorDataCount(COMPASS) + 1;
                    compassScaling = 1090;
                    
                    compassXYZ = [
                          double(typecast(data(2:3), 'int16'));
                          double(typecast(data(4:5), 'int16'));
                          double(typecast(data(6:7), 'int16'));
                        ] / compassScaling;
                    
                    % attach data to buffer
                    if mod(sensorDataCount(COMPASS), BUFFER_STEP_SIZE) == 0
                        compassBuffer = [compassBuffer; zeros(BUFFER_STEP_SIZE, 4)];
                    end
                    compassBuffer(sensorDataCount(COMPASS), 1:4) = [timestamp; compassXYZ];
                else
                    disp('unknown sensor type');
                    continue;
                end
                
                % Count the received data
                totalDataCount = totalDataCount + 1;
                if mod(totalDataCount, 1000) == 0
                    msg = sprintf('Data count: @t=%f: acc %5d, gyro %5d, compass %5d, temp %d', ...
                        toc(dataTimer), ...
                        sensorDataCount(ACCELEROMETER), ...
                        sensorDataCount(GYROSCOPE), ...
                        sensorDataCount(COMPASS), ...
                        sensorDataCount(TEMPERATURE) ...
                    );
                    disp(msg);    
                end
                       
                % Render with 30 Hz
                duration = toc(graphicsTimer);
                if duration > 1/30
                    % fetch last MARG data
                    a = accelBuffer(sensorDataCount(ACCELEROMETER), 2:4);
                    m = compassBuffer(sensorDataCount(COMPASS), 2:4);
                    
                    % Calibrate values
                    a = calibrateAccelerometer(a);
                    m = calibrateCompass(m);
                    
                    % switch axes
                    %m = [-m(1); -m(3); m(2)];
                    
                    % Debugging
                    msg = sprintf('acc: %+1.3f %+1.3f %+1.3f mag: %+1.3f %+1.3f %+1.3f', ... 
                                    a(1), a(2), a(3), m(1), m(2), m(3));
                    disp(msg);
                    
                    % normalize values
                    an = a/norm(a);
                    mn = m/norm(m);
                    
                    % plot the orientation
                    plotTrajectory(m);
                    
                    graphicsTimer = tic;
                end;
            end
            
        end
    end
    
    function cleanUp()
        disp('Cleaning up ...');
        
        % Closing the port
        fclose(s);
        delete(s);
        clear s;
        
        % Trim sensor data
        trim_sensor_data
    end
    
end