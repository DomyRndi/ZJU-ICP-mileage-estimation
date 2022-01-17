% clear;clc;

laser_map = pcread('20.ply');

%tform_init = affine3d([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);
tform_init = rigid3d([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);
robot_tf{1} = tform_init;

for i = 20:1:29
    
    %disp(i);
    
    % read
    str = [num2str(i) , '.ply'];  
    curr_ply = pcread(str);
    
    % icp

    [tform_init, curr_ply] = pcregistericp(curr_ply, laser_map, 'Metric','pointToPoint', 'InitialTransform', tform_init, 'MaxIterations', 1000, 'Tolerance', [0.00001, 0.000001]);
    robot_tf{i+1} = tform_init;
    
    % merge
    laser_map = pcmerge(laser_map, curr_ply, 0.01);
   
end

figure;
pcshow(laser_map, 'MarkerSize', 20);
hold on;
for i = 21:1:30
    fprintf("第%d处坐标位置:    ",i-20);
    fprintf("%f    ",robot_tf{i}.Translation(1));
    fprintf("%f\n",robot_tf{i}.Translation(2));
end
for i = 22:1:30
    x = [robot_tf{i-1}.Translation(1),robot_tf{i}.Translation(1)];
    y = [robot_tf{i-1}.Translation(2),robot_tf{i}.Translation(2)];
    plot(x,y);
    hold on;
end
