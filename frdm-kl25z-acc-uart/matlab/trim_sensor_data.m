disp('Trimming sensor data arrays');

% fetch descriptors
global sensorDataCount ACCELEROMETER GYROSCOPE COMPASS TEMPERATURE;

% fetch data
global accelBuffer gyroBuffer compassBuffer temperatureBuffer

% trim data
accelBuffer = accelBuffer(1:sensorDataCount(ACCELEROMETER), :);
gyroBuffer = gyroBuffer(1:sensorDataCount(GYROSCOPE), :);
compassBuffer = compassBuffer(1:sensorDataCount(COMPASS), :);
temperatureBuffer = temperatureBuffer(1:sensorDataCount(TEMPERATURE), :);

% hide descriptors
clear sensorDataCount ACCELEROMETER GYROSCOPE COMPASS TEMPERATURE;