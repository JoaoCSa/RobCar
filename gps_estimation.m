function [x_est,y_est,x_real,y_real] = gps_estimation(x,y,x_r,y_r)
    %noise_x = (-1) + (1-(-1))*rand();
    %noise_y = (-1) + (1-(-1))*rand();
    
    b = 0.5;
    noise_x = randn(1)*b;
    noise_y = randn(1)*b;
    x_est = x + noise_x;
    y_est = y + noise_y;
    x_real = x_r;
    y_real = y_r;
end