function zones = checkzones(x_in,y_in)
load('map.mat');
for i = 1:length(x_in)
    zones(i) = oc_mat(round(y_in(i)),round(x_in(i)));
end
end