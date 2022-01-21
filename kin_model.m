clear all
close all

v=5;
ws= deg2rad(-3);   %steering wheel turning speed rad/s
theta(1)=0; %car angle
phi(1)=0;   %steering wheel angle
L=2.2;

%[results]=[cos(theta)*cos(phi) 0; sin(theta)*cos(phi) 0 ;sin(phi)/L 0; 0 1]*[v;ws]
x(1)=0;
y(1)=0;
x_var = 0;
y_var = 0;
theta_var = 0;
phi_var = 0;

for i=1:30
    
    [x_var, y_var, theta_var, phi_var]=model(v,ws,theta(i),phi(i));
    
    x(i+1)=x(i)+x_var;
    y(i+1)=y(i)+y_var;
    
%     if (theta(i)+theta_var) > deg2rad(360)
%         theta(i+1)=theta(i)+theta_var - deg2rad(360);
%     elseif (theta(i)+theta_var) < deg2rad(-360)
%         theta(i+1)=theta(i)+theta_var + deg2rad(360);
%     else 
%         theta(i+1)=theta(i)+theta_var;
%     end
    
    theta(i+1)=theta(i)+theta_var;
    
    if abs(phi(i)+phi_var) > deg2rad(45)
        phi(i+1) = phi(i);
    else 
        phi(i+1)=phi(i)+phi_var;
    end
        
end

figure
plot(x,'b')
hold on 
plot(y,'c')
plot(rad2deg(theta),'g')
plot(rad2deg(phi),'r')
legend("x coordinate","y coordinate","car angle", "wheel angle")
grid on

figure
plot(x,y,'c')
grid on
%% simple model of the car

function [x_dot, y_dot, theta_dot, phi_dot] = model(v,ws,theta,phi)
    L=2.2;
    [results]=[cos(theta)*cos(phi) 0; sin(theta)*cos(phi) 0 ;sin(phi)/L 0; 0 1]*[v;ws]
    x_dot=results(1);
    y_dot=results(2);
    theta_dot=results(3);
    phi_dot=results(4);
end 

