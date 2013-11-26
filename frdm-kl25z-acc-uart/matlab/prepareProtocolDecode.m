function prepareProtocolDecode
        
    % Definitions
    global SOH EOT ESC ESC_XOR DA TA
    SOH     = uint8(1);
    EOT     = uint8(4);
    ESC     = uint8(27);
    ESC_XOR = uint8(66);
    DA      = uint8(218);
    TA      = uint8(122);

    % Detection state machine
    % 0 = searching for preamble --> 0
    % 1 = preamble found, expecting SOH --> 1
    % 2 = SOH found, expecting length --> 2
    % 3 = length found, expecting data --> 4
    % 4 = reading bytes --> 5
    % 5 = expecting EOT --> 0
    global state
    state = 0;
    
    % Decoding variables
    global dataLength data dataBytesRead escapeDetected dataReady lastByte
    lastByte = NaN;
    dataLength = 0;
    data = [];
    dataBytesRead = 0;
    escapeDetected = false;
    dataReady = false;
    
    % Data counters
    global sensorDataCount ACCELEROMETER GYROSCOPE COMPASS TEMPERATURE;
    ACCELEROMETER = 1;
    GYROSCOPE     = 2;
    COMPASS       = 3;
    TEMPERATURE   = 4;
    sensorDataCount = zeros(4,1);
end