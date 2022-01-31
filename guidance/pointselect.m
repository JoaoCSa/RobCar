function [pi,pf]=pointselect()

load("map.mat");
load("graph.mat");

imshow(map)
title('Click on image to select intial and final positions.')
hold on
plot(G,'XData',x,'YData',y,'NodeLabelColor','w','NodeColor','w','EdgeColor','w')
hold on
% plot(black_coord(2,:),black_coord(1,:),'w.');
% plot(red_coord(2,:),red_coord(1,:),'r.');
% plot(green_coord(2,:),green_coord(1,:),'g.');
% plot(blue_coord(2,:),blue_coord(1,:),'b.');

valid = 0;

while valid ~= 1
    figure(1)
    imshow(map)
    hold on
    plot(G,'XData',x,'YData',y,'NodeLabelColor','w','NodeColor','w','EdgeColor','w');

    % Select starting point
    [xi,yi] = ginput(1);
    hold on
    plot(xi,yi,'r.');
    text(xi,yi,'IP');

    % Select end point
    [xf,yf] = ginput(1);
    hold on
    plot(xf,yf,'g.');
    text(xf,yf,'FP');

    % Checks if the points belong to free space
    if (oc_mat(round(yf),round(xf)) ~= 1) && (oc_mat(round(yi),round(xi)) ~= 1)
        valid = 1;
    else
        msgbox('Please select positions again. Make sure to select points on valid areas. Press ENTER to continue.', 'Error','error');
        pause();
%         fprintf('Please select positions again. Make sure to select points on valid areas. \n')
    end
end

pi = [xi,yi];

pf = [xf,yf];

end