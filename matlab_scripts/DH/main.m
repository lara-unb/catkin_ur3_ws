function [lh, c, r] = main( folder, num_joints, plot )

for i=1:num_joints
    str = strcat('j', num2str(i));
    str = strcat(str,'.csv');
    fstr = strcat(folder,'/');
    T{i} = load(strcat(fstr,str));
    
    if(size(T{i}, 2) == 4)
       %Contains time in first row
       T{i} = T{i}(:,2:4);
    end
end

% center and normals to each joint
c = zeros(3,num_joints);
n = zeros(3,num_joints);
r = zeros(1,num_joints);

if(plot)
    % Plot the saved points and the corresponding best-fitting-circles
    figure();
    hold on
    % axis equal
    grid on
end

% color of the plots (up to 7)
    colors = [1 0 0;
    0 1 0;
    0 0 1;
    0 1 1;
    1 1 0;
    1 0 1];

% plot handle vector
lh = gobjects(num_joints,1);
% determine the center and normal vector for each joint
for i=1:num_joints
    
    [c(:,i), n(:,i), lh(i), r(i)] = best_fitting_circle(T{i}, plot, colors(i,:));
    
end
n(:,6) = [0;0;1];
if(plot)
    for i=1:num_joints
        
        mArrow3(c(:,i), c(:,i)+ 0.2*n(:,i), 'color', colors(i,:), ...
            'tipWidth', 0.01, 'stemWidth', 0.005);
        scatter3(c(1,i), c(2,i), c(3,i), 'filled', 'CData', colors(i,:));
    end
    hold off 
end

title(folder)
xlabel('X')
ylabel('Y')
zlabel('Z')
%legend(lh, 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7');
xlim([-0.8 0.6])
ylim([-0.8 0.6])
zlim([-0.7 1.0])
view(70,20)

% If you want to export a figure
%exp_fig_name = strcat(folder,'.png');
%export_fig exp_fig_name -m2 -nocrop -svg

disp('done')
