buffer_length = 1000;
fs = 200;
sam_per_read = 10;
f_range = (0:buffer_length-1)*(fs/buffer_length);

global ax_data;
global ay_data;
global az_data;
global buffer_available;

an_ax_fft = animatedline('MaximumNumPoints',f_range);
an_ay_fft = animatedline('MaximumNumPoints',f_range);
an_az_fft = animatedline('MaximumNumPoints',f_range);

while 1
    if buffer_available
        buffer_available = false;
        ax_fft = fft(ax_data);
        ay_fft = fft(ay_data);
        az_fft = fft(az_data);
        ax_power = abs(ax_fft).^2/buffer_length;
        ay_power = abs(ay_fft).^2/buffer_length;
        az_power = abs(az_fft).^2/buffer_length;
        %绘图
        addpoints(an_ax_fft,f_range,ax_power);
        addpoints(an_ay_fft,f_range,ay_power);
        addpoints(an_az_fft,f_range,az_power);
    end
    pause(1/fs*sam_per_read);
end