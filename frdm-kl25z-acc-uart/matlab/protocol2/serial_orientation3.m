function serial_orientation3
    % SERIAL_TEST Reads streamed sensor data.
    %   define  
    %   global sensorDataCount accelBuffer gyroBuffer compassBuffer temperatureBuffer
    %   at the workspace after running this function to get the data.

    close all; clear all; clc;
    
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
    
    % Prepare the plot
    % preparePlotOrientation();    
    
    % Definitions
    global dataReady data
    prepareProtocolDecode();
            
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
                timestamp = toc(dataTimer);
                
                % Attach NaN byte to circular buffer to aid debugging
                %byteCircBuf = [byteCircBuf(2:end), NaN];
                
                % Skip everything that is not from the fused sensor
                type = data(1);
                if type == 43
                   
                    % Decode  data
                    scaling = (1/65535);
                    
                    quat = [
                        double(typecast(data(2:5), 'int32'));
                        double(typecast(data(6:9), 'int32'));
                        double(typecast(data(10:13), 'int32'));
                        double(typecast(data(14:17), 'int32'));
                        ] * scaling;
                elseif type == 42
                    disp('Please buy the commercial version to enable this feature.');
                    continue;
                    
                    % Decode  data
                    scaling = (1/65535) * 180 / pi;
                    
                    rpy = [
                        double(typecast(data(2:5), 'int32'));
                        double(typecast(data(6:9), 'int32'));
                        double(typecast(data(10:13), 'int32'));
                        ] * scaling;
                else
                    disp('unknown sensor type');
                    continue;
                end
                
                % Render with 30 Hz
                duration = toc(graphicsTimer);
                if duration > 1/30                    

                    % Quaternion to DCM
                    DCM = quaternionToRotation(quat);

                    %{
                    % extract angles
                    % see: William Premerlani, "Computing Euler Angles from Direction Cosines"
                    pitch = -asind(DCM(3, 1));
                    roll  =  atan2d(DCM(3, 2), sign(DCM(3,3))*sqrt(DCM(3, 1)^2 + DCM(3, 3)^2));
                    yaw   =  atan2d(DCM(2, 1), DCM(1, 1));
                    %}
                    
                    [roll, pitch, yaw] = quaternionToEuler(quat);

                    % Debugging
                    fprintf('rpy: %+1.3f %+1.3f %+1.3f\n', roll, pitch, yaw);
                                
                    % plot the orientation
                    %plotOrientation(DCM', [NaN NaN NaN], [NaN NaN NaN]);

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