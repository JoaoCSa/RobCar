function [x_dot, y_dot, theta_dot] = model_ss(x,y,v,ws,theta)
    
    A = eye(3);
    B = [cos(theta), 0; sin(theta), 0; 0, 1];
    u_vec = [v; ws];
    state = [x; y; theta];    
    
    state = A*state + B*u_vec;
    
    x_dot=state(1);
    y_dot=state(2);
    theta_dot=state(3);
    
end 