disp('Saving sensor data arrays');

% fetch data
global accelBuffer gyroBuffer compassBuffer temperatureBuffer

save mpu6050_accelerometer.mat accelBuffer
save mpu6050_gyroscope.mat gyroBuffer
save mpu6050_temperature.mat temperatureBuffer
save hmc5883l_compass.mat compassBuffer