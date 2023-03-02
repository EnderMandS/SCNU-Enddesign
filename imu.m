clear all;
ESP_obj = arduino("COM7",'ESP32-WROOM-DevKitC',"Libraries","I2C");
scanI2CBus(ESP_obj);
imu_obj = mpu6050(ESP_obj,'OutputFormat','matrix','TimeFormat','duration','I2CAddress',0x68,SampleRate=200,SamplesPerRead=10);

an_ax = animatedline('MaximumNumPoints',2000,'Color',"#0072BD");
an_ay = animatedline('MaximumNumPoints',2000,'Color',"#D95319");
an_az = animatedline('MaximumNumPoints',2000,'Color',"#EDB120");
% an_gx = animatedline('MaximumNumPoints',2000,'Color',"#7E2F8E");
% an_gy = animatedline('MaximumNumPoints',2000,'Color',"#4DBEEE");
% an_gz = animatedline('MaximumNumPoints',2000,'Color',"#A2142F");
while 1
    [accel, gyro, timeStamps, overrun] = imu_obj.read;
    duration_ms = milliseconds(timeStamps);
    ax = accel(:,1);
    ay = accel(:,2);
    az = accel(:,3);
    gx = gyro(:,1);
    gy = gyro(:,2);
    gz = gyro(:,3);
    addpoints(an_ax,duration_ms,ax);
    addpoints(an_ay,duration_ms,ay);
    addpoints(an_az,duration_ms,az);
%     addpoints(an_gx,duration_ms,gx);
%     addpoints(an_gy,duration_ms,gy);
%     addpoints(an_gz,duration_ms,gz);
    drawnow
end