clear all
% 参数定义
global sam_per_read;
global buffer_length;
sam_per_read = 10;
buffer_length = 400;
fs = 200;
f_range = (0:buffer_length-1)*(fs/buffer_length);
fshift = (-buffer_length/2:buffer_length/2-1)*(fs/buffer_length);
an_mode = 3; % 1acceleration 2fft 3eular

% 创建数据缓冲区
ax_data = zeros(1,buffer_length);
ay_data = zeros(1,buffer_length);
az_data = zeros(1,buffer_length);

% 初始化硬件
if ~exist("ESP_obj","var")
    disp("Connecting to ESP32");
    ESP_obj = arduino("COM7",'ESP32-WROOM-DevKitC',"Libraries","I2C");
end
clear imu_obj;
imu_obj = mpu6050(ESP_obj,'OutputFormat','matrix','TimeFormat','duration','I2CAddress',0x68,SampleRate=fs,SamplesPerRead=sam_per_read);
disp("MPU Connected");

% 定义动态绘图结构体
switch an_mode
    case 1
        disp("Running at normal mode");
        an_ax = animatedline('MaximumNumPoints',2000,'Color',"#0072BD");
        an_ay = animatedline('MaximumNumPoints',2000,'Color',"#D95319");
        an_az = animatedline('MaximumNumPoints',2000,'Color',"#EDB120");
        % an_gx = animatedline('MaximumNumPoints',2000,'Color',"#7E2F8E");
        % an_gy = animatedline('MaximumNumPoints',2000,'Color',"#4DBEEE");
        % an_gz = animatedline('MaximumNumPoints',2000,'Color',"#A2142F");
        drawnow
        title('Acceleration');
        xlabel("t/ms");
        ylabel("m^2/s");
        legend('ax','ay','az');
    case 2
        disp("Running at fft mode");z
        an_ax_fft = animatedline('MaximumNumPoints',length(fshift),'Color',"#0072BD");
        an_ay_fft = animatedline('MaximumNumPoints',length(fshift),'Color',"#D95319");
        an_az_fft = animatedline('MaximumNumPoints',length(fshift),'Color',"#EDB120");
        drawnow
        title('FFT');
        xlabel("f (Hz)");
        ylabel("|P(f)|");
        legend('ax','ay','az');
    case 3
        disp("Running at eular mode");
        an_yaw = animatedline('MaximumNumPoints',2000,'Color',"#0072BD");
        an_pitch = animatedline('MaximumNumPoints',2000,'Color',"#D95319");
        an_roll = animatedline('MaximumNumPoints',2000,'Color',"#EDB120");
        fuse = complementaryFilter("SampleRate",200,"HasMagnetometer",false,"OrientationFormat","quaternion");
        drawnow
        title('Eular Angle');
        xlabel("t/s");
        ylabel("degree");
        legend('yaw','pitch','roll');
end

data_cnt = 0;
buffer_available = false;
while 1
    [accel, gyro, timeStamps, ~] = imu_obj.read;



    % 数据转换
    duration_s = seconds(timeStamps);
    ax_t = accel(:,1);
    ay_t = accel(:,2);
    az_t = accel(:,3);
    gx_t = gyro(:,1);
    gy_t = gyro(:,2);
    gz_t = gyro(:,3);

    % 数据存入缓冲区
    ax_data = dataShiftAppend(ax_data,ax_t);
    ay_data = dataShiftAppend(ay_data,ay_t);
    az_data = dataShiftAppend(az_data,az_t);
    data_cnt = data_cnt + 1;
    if data_cnt>(buffer_length/sam_per_read)
        buffer_available = true;
    end

    %动态绘图
    switch an_mode
        case 1
            addpoints(an_ax,duration_s,ax_t);
            addpoints(an_ay,duration_s,ay_t);
            addpoints(an_az,duration_s,az_t);
%             addpoints(an_gx,duration_ms,gx_t);
%             addpoints(an_gy,duration_ms,gy_t);
%             addpoints(an_gz,duration_ms,gz_t);
        case 2
            if buffer_available
                ax_fft = fftshift(fft(ax_data));
                ay_fft = fftshift(fft(ay_data));
                az_fft = fftshift(fft(az_data));
                ax_power = abs(ax_fft).^2/buffer_length;
                ay_power = abs(ay_fft).^2/buffer_length;
                az_power = abs(az_fft).^2/buffer_length;
                %绘图
                addpoints(an_ax_fft,fshift,ax_power);
                addpoints(an_ay_fft,fshift,ay_power);
                addpoints(an_az_fft,fshift,az_power);
            end
        case 3
            [orientation_q,~] = fuse(accel,gyro);
            eulerAngles = euler(orientation_q,'ZYX','point');
            eulerAngles = rad2deg(eulerAngles);
            addpoints(an_yaw    ,duration_s,eulerAngles(:,1));
            addpoints(an_pitch  ,duration_s,eulerAngles(:,2));
            addpoints(an_roll   ,duration_s,eulerAngles(:,3));
    end
    drawnow
end

function data_source = dataShiftAppend(data_source,data_append)
    global sam_per_read;
    global buffer_length;
    data_source = circshift(data_source,-sam_per_read);
    data_source(1,buffer_length-sam_per_read+1:buffer_length) = data_append;
end

