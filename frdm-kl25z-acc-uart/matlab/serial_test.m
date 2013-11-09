function serial_test
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
    global s;
    s = serial('COM4', ...
        'FlowControl', 'none', ...
        'BaudRate', 125000, ...
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
    fopen(s);

    % Prepare plot
    figureHandle = figure('NumberTitle', 'off', ...
        'Name', 'MMA8451Q Output', ...
        'Color', [0.027 0.211 0.259], ...
        'Visible', 'off' ...
        );
    
    % Prepare axes
    axesHandle = axes('Parent', figureHandle, ...
        'XGrid', 'on', ...
        'XColor', [0.973 0.973 0.973], ...
        'YGrid', 'on', ...
        'YColor', [0.973 0.973 0.973], ...
        'ZGrid', 'on', ...
        'ZColor', [0.973 0.973 0.973], ...
        'Color', [0.1 0.1 0.1], ...
        'ZDir', 'reverse' ...
        );

    % Prepare the data array
    xtrack = NaN(1, 10);
    ytrack = NaN(1, 10);
    ztrack = NaN(1, 10);
    
    % Prepare the plot
    hold on;
    plotHandle = line(NaN, NaN, NaN, ...
        'Parent', axesHandle, ...
        'Marker', 'v', ...
        'LineWidth', 2, ...
        'MarkerSize', 20, ...
        'Color', [0.972 0.149 0.427] ...
        );
    trackHandle = line(xtrack, ytrack, ztrack, ...
        'Parent', axesHandle, ...
        'LineStyle', ':', ...
        'Marker', '.', ...
        'LineWidth', 1, ...
        'MarkerSize', 1, ...
        'Color', [0.972 0.149 0.427] ...
        );
    
    %set(plotHandle(1), 'MarkerSize', 20);
    
    xlim(axesHandle, [-1 1]);
    ylim(axesHandle, [-1 1]);
    zlim(axesHandle, [-1 1]);
    
    view(axesHandle, -37.5, 30);
    axis square;
    set(figureHandle, 'Visible', 'on');
    
    % Grab jobject for fread
    sjobject = igetfield(s, 'jobject');
        
    % Definitions
    SOH     = uint8(1);
    STX     = uint8(2);
    ETX     = uint8(3);
    EOT     = uint8(4);
    ESC     = uint8(27);
    ESC_XOR = uint8(66);
    DA      = uint8(218);
    TA      = uint8(122);

    % Detection state machine
    % 0 = searching for preamble --> 0
    % 1 = preamble found, expecting SOH --> 1
    % 2 = SOH found, expecting length --> 2
    % 3 = length found, expecting STX --> 3
    % 4 = reading bytes --> 4
    % 5 = bytes read, expecting ETX --> 5
    % 6 = ETX found, expecting EOT --> 0
    state = 0;
    
    % Decoding variables
    dataLength = 0;
    data = [];
    dataBytesRead = 0;
    escapeDetected = false;
    dataReady = false;
    
    % Debugging
    %byteCircBuf = NaN(1, 24);
    
    % Start timing for the graphics loop
    tic;
    
    % Reading the data
    bulkSize = 80;
    byteIndices = 1:bulkSize;
    lastByte = 0;
    while true
        % Read bytes; A profiler run showed that fread(s, count) is
        % horribly slow mainly due to error string formattings even
        % if they are not required. Because of this, a more low-level
        % variant of the function is called.
        out = fread(sjobject, bulkSize, 0, 0); % 0, 0 meaning unsigned int 8
        bytes = typecast(out(1), 'uint8');     % unfortunately it is not unsigned
        
        for b=byteIndices
            byte = bytes(b);
            
            % Attach byte to circular buffer
            %byteCircBuf = [byteCircBuf(2:end), double(byte)];

            % Decode the protocol
            protocolDecode(byte);

            % Decode data buffer
            if dataReady
                % Attach NaN byte to circular buffer to aid debugging
                %byteCircBuf = [byteCircBuf(2:end), NaN];
                
                xyz = [
                    double(typecast(data(1:2), 'int16'));
                    double(typecast(data(3:4), 'int16'));
                    double(typecast(data(5:6), 'int16'));
                    ] / 4096;
                
                %msg = sprintf('x: %+1.5f  y: %+1.5f  z: %+1.5f', xyz(1), xyz(2), xyz(3));
                %disp(msg);
                    
                % Render with 60 Hz
                duration = toc;
                if duration > 1/60
                    xtrack = [xtrack(2:end) xyz(1)];
                    ytrack = [ytrack(2:end) xyz(2)];
                    ztrack = [ztrack(2:end) xyz(3)];
                    
                    set(plotHandle, 'XData', xyz(1), 'YData', xyz(2), 'ZData', xyz(3));
                    set(trackHandle, 'XData', xtrack, 'YData', ytrack, 'ZData', ztrack);
                    
                    xlabel(num2str(xyz(1)));
                    ylabel(num2str(xyz(2)));
                    zlabel(num2str(xyz(3)));
                    
                    drawnow;
                    tic;
                end;
                
                % Thanks, but no.
                dataReady = false;
            end
            
        end
    end
    
    function protocolDecode(byte)       
        % Switch states
        switch state
            % Await preamble
            case 0
                if byte == DA
                    %state = 0;
                elseif (byte == TA) && (lastByte == DA)
                    % preamble detected
                    state = 1;
                else
                    %state = 0;
                    
                    %disp('protocol error in state 0');
                    %disp(s.BytesAvailable);
                end
                
                % Remember byte for protocol decoding
                lastByte = byte;
            
            % Await SOH
            case 1
                if byte == SOH
                    state = 2;
                else
                    state = 0;
                    disp('protocol error in state 1');
                end
                
            % Read length
            case 2
                oldDataLength = dataLength;
                dataLength = byte;
                dataBytesRead = 0;
                if oldDataLength ~= dataLength
                    data = zeros(dataLength, 1, 'uint8');
                end
                state = 3;
                
            % Await SOH
            case 3
                if byte == STX
                    state = 4;
                else
                    state = 0;
                    disp('protocol error in state 3');
                end
                
            % Read data bytes
            case 4
                if byte == ESC
                    escapeDetected = true;
                    %state = 4;
                else
                    if escapeDetected
                        byte = xor(byte, ESC_XOR);
                        escapeDetected = false;
                    end

                    dataBytesRead = dataBytesRead + 1;
                    data(dataBytesRead) = byte;
                    
                    if dataBytesRead == dataLength
                        state = 5;
                    else
                        %state = 4;
                    end
                end
                
            % Await ETX
            case 5
                if byte == ETX
                    state = 6;
                else
                    state = 0;
                    disp('protocol error in state 5');
                end
                
            % Await EOT
            case 6
                if byte == EOT
                    state = 0;
                    dataReady = true;
                else
                    state = 0;
                    disp('protocol error in state 6');
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