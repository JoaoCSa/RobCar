clear all
close all

v=5;                    % constant speed
ws= deg2rad(-0.02);        % steering wheel turning speed rad/s
theta(1)=0;             % initial car angle
phi(1)=0;               % initial steering wheel angle
L=2.2;

%[results]=[cos(theta)*cos(phi) 0; sin(theta)*cos(phi) 0 ;sin(phi)/L 0; 0 1]*[v;ws]
x(1)=0;                 % initial position
y(1)=0;                 % initial position
x_var = 0;              % initial position variation
y_var = 0;              % initial position variation
theta_var = 0;          % initial car angle variation
phi_var = 0;            % initial steering wheel angle variation
x_real(1) = 0;
y_real(1) = 0;
x_r(1) = 0;
y_r(1) = 0;

for i=1:200
    if (i < 50) || (i > 150) 
        [x(i),y(i),x_real(i),y_real(i)]=gps_estimation(x(i),y(i),x_real(i),y_real(i));
        [x_var, y_var, theta_var, phi_var]=model(v,ws,theta(i),phi(i));
        
        x_real(i+1) =  x_real(i) + x_var;
        y_real(i+1) =  y_real(i) + y_var; 

        x(i+1) = x(i) + x_var;
        y(i+1) = y(i) + y_var;
    else 
        [x_var, y_var, theta_var, phi_var]=model(v,ws,theta(i),phi(i));
        
        x_real(i+1) =  x_real(i) + x_var;
        y_real(i+1) =  y_real(i) + y_var; 

        x(i+1) = x(i) + x_var;
        y(i+1) = y(i) + y_var;
    end
    
    
    
%     if (theta(i)+theta_var) > deg2rad(360)
%         theta(i+1)=theta(i)+theta_var - deg2rad(360);
%     elseif (theta(i)+theta_var) < deg2rad(-360)
%         theta(i+1)=theta(i)+theta_var + deg2rad(360);
%     else 
%         theta(i+1)=theta(i)+theta_var;
%     end
    
    theta(i+1) = theta(i) + theta_var;
    
    if abs(phi(i) + phi_var) > deg2rad(45)
        phi(i+1) = phi(i);
    else 
        phi(i+1) = phi(i) + phi_var;
    end
        
end

figure

plot([1:49],x(1,[1:49]),'b')
hold on 
plot([49:151],x(1,[49:151]),'color','[0.6350 0.0780 0.1840]')
plot([151:200],x(1,[151:200]),'b')
plot(x_real, 'c')

plot([1:49],y(1,[1:49]),'g')
plot([49:151],y(1,[49:151]),'color','[0.6350 0.0780 0.1840]')
plot([151:200],y(1,[151:200]),'g')
plot(y_real, 'color','[0.4660 0.6740 0.1880]')

xline(49)
xline(151)
%plot(rad2deg(theta),'g')
%plot(rad2deg(phi),'r')
legend("x coordinate","-","-","real x coordinate","y coordinate","-","-", "real y coordinate")
ylabel('[m]')
xlabel('time')
grid on

figure
plot(x,y,'c')
hold on
plot(x_real,y_real,'color','[0.4660 0.6740 0.1880]')
legend("car estimated global position","car real global position")
ylabel('y[m]')
xlabel('x[m]')
grid on