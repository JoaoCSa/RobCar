x=1730;
y=2051;

k = 0.1852;
x_mat = x*1;
y_mat = y*1;

%oc_matx = load('oc_mat.mat')

%global oc_matx.oc_mat(round(x_mat),round(y_mat))
% [x_var_r, x_var_c] = find(oc_matx.green_coord(1,:) == round(x_mat))
% size(x_var_c)
% 
% 
% for i = 1:size(x_var_c)
%     if oc_matx.green_coord(2,x_var_c(i)) == round(y_mat)
%         'hello'
%         break
%     else 
%         'goodbye'
%     end
% end
% 
%[y_var_r, y_var_c] = find(oc_matx.green_coord(2,:) == round(y_mat))




% x_var = find(green_x.data == round(x_mat));
% y_var = find(green_y.data == round(y_mat));
% 
% if (~isempty(x_var) && ~isempty(y_var))
%     'hello' 
% else
%     'ded'
% end
% 
% length(z_var)
% 
% val = intersect(x_var,z_var)
% if (~isempty(val))
%     if (green_x.data(val) == round(x_mat) && green_y.data(val) == round(y_mat))
%         z = 0;
%     else 
%         z = 1;
%     end
% end
% 


z = stepImpl(x,y,green_x,green_y)




function z = stepImpl(x,y,green_x,green_y)
    % Implement algorithm. Calculate y as a function of input u and
    % discrete states.

    k = 0.1852;
    x_mat = x*1
    y_mat = y*1

    x_var = find(green_x == round(x_mat))
    y_var = find(green_y == round(y_mat))

    if (~isempty(x_var) && ~isempty(y_var))

        val = intersect(x_var,y_var)

        if (~isempty(val))
            if (green_x.data(val) == round(x_mat) && green_y.data(val) == round(y_mat))
                z = 0;
            else 
                z = 1;
            end
        end
    else
        z = 1;
    end

end





















% x_var = find(oc_matx.green_coord(1,:) == round(x_mat));
% y_var = find(oc_matx.green_coord(2,:) == round(y_mat));
% [val,pos] = intersect(x_var,y_var)
% 
% if (oc_matx.green_coord(1,val) == round(x_mat) && oc_matx.green_coord(2,val) == round(y_mat))
%     z = 0;
% else 
%     z = 1;
% end

%y_var = find(oc_matx.green_coord == round(y_mat))

%[row,col]=find(oc_matx.green_coord == round(x_mat));

% if isempty(y_var) && isempty(x_var)
%     'rip'
% else 
%     'hello'
% end
