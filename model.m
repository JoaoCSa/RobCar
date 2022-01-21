function [x_dot, y_dot, theta_dot, phi_dot] = model(v,ws,theta,phi)
    L=2.2;
    [results]=[cos(theta) 0; sin(theta) 0 ;tan(phi)/L 0; 0 1]*[v;ws];
    x_dot=results(1);
    y_dot=results(2);
    theta_dot=results(3);
    phi_dot=results(4);
end 