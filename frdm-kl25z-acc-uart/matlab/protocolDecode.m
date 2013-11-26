function [availableData] = protocolDecode(byte)
    global SOH EOT ESC ESC_XOR DA TA
    global state
    global dataLength data dataReady dataBytesRead escapeDetected lastByte

    availableData = 0;
    
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
            dataLength = byte;
            dataBytesRead = 0;
            data = zeros(dataLength, 1, 'uint8');
            state = 3;

        % Read data bytes
        case 3
            if byte == ESC
                escapeDetected = true;
                %state = 3;
            else
                if escapeDetected
                    byte = bitxor(byte, ESC_XOR);
                    escapeDetected = false;
                end

                dataBytesRead = dataBytesRead + 1;
                data(dataBytesRead) = byte;

                if dataBytesRead == dataLength
                    state = 4;
                else
                    %state = 3;
                end
            end

        % Await EOT
        case 4
            if byte == EOT
                state = 0;
                dataReady = true;
                availableData = dataBytesRead;
            else
                state = 0;
                disp('protocol error in state 6');
            end
    end
end