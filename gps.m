x=5;
y=3;

[x,y]=gps_estimation(x,y)

function [x_est,y_est] = gps_estimation(x,y)
    noise_x = (-1) + (1-(-1))*rand();
    noise_y = (-1) + (1-(-1))*rand();
    x_est = x + noise_x;
    y_est = y + noise_y;
end
