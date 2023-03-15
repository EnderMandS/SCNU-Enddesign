% 将串口读取到的数据存入文件
clear;
readtime = 10; % 秒

com = serialport("COM8",115200,"Parity","even","Timeout",1);

time_start = datetime('now');
while seconds(datetime('now')-time_start)<readtime
    read_data = read(com,com.NumBytesAvailable,"uint8");
    frame_end_index = strfind(read_data,'END');
    l_frame_end_index = length(frame_end_index);
    if l_frame_end_index>1
        for i=1:l_frame_end_index
            if i==l_frame_end_index
                break
            end
            frame = read_data((l_frame_end_index(i)+3):(l_frame_end_index(i+1)-1));
            frameProcess(frame);
        end
    end
end
clear com

filename = strcat('data/',string(datetime('now','Format','uuuu-MM-dd-HH-mm-ss')),'.mat');
save(filename,"accel","gyro");


function [accel,gyro,time] = frameProcess(frame)
    % mpu_data = mpu_obj.get_raw_values() #14
    % t = time.ticks_ms().to_bytes(4, 'little') #4
    % s = sum(mpu_data + t).to_bytes(1, 'little') #1
    % data = mpu_data + t + s + b'END' # 14+4+1+3=22
    if rem(sum(frame(1:18)),256)==frame(19)
        time = frame(15)+frame(16)*256+frame(17)*65536+frame(18)*16777216;
        accel = zeros(1,3,'double');
        gyro = zeros(1,3,'double');
    else
        time=0;
    end
end