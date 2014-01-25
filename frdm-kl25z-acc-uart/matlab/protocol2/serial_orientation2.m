function serial_orientation2
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
    
    % Definitions
    global dataReady data
    prepareProtocolDecode();
    
    % Data counters
    global sensorDataCount RPY
    RPY = 1;
    sensorDataCount = zeros(4,1);
    totalDataCount = 0;
    
    % data buffers
    BUFFER_STEP_SIZE = 1000;
    global rpyBuffer
    rpyBuffer         = zeros(BUFFER_STEP_SIZE, 3); % xyz
        
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
                   
                    % Decode MMA8451Q data
                    sensorDataCount(RPY) = sensorDataCount(RPY) + 1;
                    scaling = (1/65535) * 180 / pi;
                    
                    rpy = [
                        double(typecast(data(2:5), 'int32'));
                        double(typecast(data(6:9), 'int32'));
                        double(typecast(data(10:13), 'int32'));
                        ] * scaling;

                    % attach data to buffer
                    if mod(sensorDataCount(RPY), BUFFER_STEP_SIZE) == 0
                        rpyBuffer = [rpyBuffer; zeros(BUFFER_STEP_SIZE, 3)];
                    end
                    rpyBuffer(sensorDataCount(RPY), 1:3) = rpy;
                else
                    disp('unknown sensor type');
                    continue;
                end
                
                % Count the received data
                totalDataCount = totalDataCount + 1;
                if mod(totalDataCount, 1000) == 0
                    msg = sprintf('Data count: @t=%f: rpy %5d', ...
                        toc(dataTimer), ...
                        sensorDataCount(RPY) ...
                    );
                    disp(msg);    
                end
                       
                % Render with 30 Hz
                duration = toc(graphicsTimer);
                if duration > 1/30
                    % fetch last data
                    rpy = rpyBuffer(sensorDataCount(RPY), 1:3);
                    
                    % Debugging
                    msg = sprintf('rpy: %+1.3f %+1.3f %+1.3f', ... 
                                    rpy(1), rpy(2), rpy(3));
                    disp(msg);
                   
                    %{
                    % fetch orientation
                    [~, ~, ~, DCM, coordinateSystem] = yawPitchRoll(a, m);
                    
                    % plot the orientation
                    plotOrientation(DCM', coordinateSystem, an, mn);
                    %}

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
    end
    
end