function [x_out,y_out] = smoothenpath(x_in,y_in,d)
z = linspace(0, 1, numel(x_in));
m = round(d); 
zz = linspace(0, 1, m);
x_out = pchip(z, x_in, zz); 
y_out = pchip(z, y_in, zz);
end