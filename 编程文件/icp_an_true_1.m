clc,clear
%format short
format long

flag = 1;   %列号，用于迭代
T_final_add=eye(4,4);   %迭代矩阵
Rf_remin{1}=eye(3,3);
Tf_remin{1}=[0,0,0]';
for i=1:1:9
    number_flag = i;
    Rf_sum=eye(3,3);    %初始化旋转矩阵
    Tf_sum=[0,0,0]';    %初始化位移矩阵
    fprintf('ICP  %d-->%d',i-1);
    fprintf('-->%d\n',i);
    str0 = [num2str(i-1) , '.ply'];
    str1 = [num2str(i) , '.ply'];
    ply_0 = pcread(str0);
    ply_1 = pcread(str1);
    p0 = getmat(str0);  %读取点云文件数据以矩阵形式存入p0
    p1 = getmat(str1);
    name=[str0,'与',str1,'的初始点云数据'];
    ShowData(p0,p1,name);   %显示未匹配的点云地图
    
    data_source=p0';
    data_target=p1';
    T_final=eye(4,4);   %旋转矩阵初始值
    iteration=0;    %记录迭代次数
    Rf=T_final(1:3,1:3);
    Tf=T_final(1:3,4);
    data_target=Rf*data_target+Tf*ones(1,size(data_target,2));    %初次更新点集（代表粗配准结果）
    err=1;
    while(err>0.01)
        iteration=iteration+1;    %迭代次数
        %利用欧式距离找出对应点集
        k=size(data_target,2);
        for i = 1:k
            data_q1(1,:) = data_source(1,:) - data_target(1,i);    % 两个点集中的点x坐标之差
            data_q1(2,:) = data_source(2,:) - data_target(2,i);    % 两个点集中的点y坐标之差
            data_q1(3,:) = data_source(3,:) - data_target(3,i);    % 两个点集中的点z坐标之差
            distance = data_q1(1,:).^2 + data_q1(2,:).^2 + data_q1(3,:).^2;  % 欧氏距离
            [min_dis, min_index] = min(distance);   % 找到距离最小的那个点
            data_mid(:,i) = data_source(:,min_index);   % 将那个点保存为对应点
            error(i) = min_dis;     % 保存距离差值
        end
        %去中心化
        data_target_mean=mean(data_target,2);   %得到点集质心位置
        data_mid_mean=mean(data_mid,2);
        data_target_c=data_target-data_target_mean*ones(1,size(data_target,2)); %得到去质心坐标矩阵
        data_mid_c=data_mid-data_mid_mean*ones(1,size(data_mid,2));
        %SVD分解
        W = data_target_c*data_mid_c';
        [U,S,V]=svd(W);
        %根据线性代数方法求解Rf和Tf
        Rf=V*U';
        Rf_sum=Rf*Rf_sum;
        Tf=data_mid_mean-Rf*data_target_mean;
        Tf_sum=Tf_sum+Tf;
        err=mean(error);
        T_t=[Rf';Tf'];
        column = [0;0;0;1];
        T_t=[T_t column];
        T_final=T_t*T_final;   %更新旋转矩阵
        
        data_target=Rf*data_target+Tf*ones(1,size(data_target,2));    %更新点集
        if iteration>=200
            break
        end
        [tform, ply_1] = pcregistericp(ply_1, ply_0);   %采用demo提供方法求解旋转矩阵
        T_error = T_final - tform.T;    %作差比较误差
    end
    
    disp(['误差err=',num2str(err)]);
    
    name=[str0,'与',str1,'ICP操作后点云数据'];
    ShowData(p0,data_target',name);     %画出匹配后的点云地图
    axis equal
    Rf_remin{flag}=Rf_sum;    %记录每次匹配的旋转矩阵
    Tf_remin{flag}=Tf_sum;
    flag=flag+1;
end

laser_map = pcread('0.ply');
tform_init = rigid3d([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);
robot_tf{1} = tform_init;

%采用demo方法得到每两个数据间的旋转矩阵
for i = 0:1:9
    % read
    str = [num2str(i) , '.ply'];
    curr_ply = pcread(str);
    % icp
    [tform_init, curr_ply] = pcregistericp(curr_ply, laser_map, 'Metric','pointToPoint', 'InitialTransform', tform_init, 'MaxIterations', 1000, 'Tolerance', [0.00001, 0.000001]);
    robot_tf{i+1} = tform_init;
    % merge
    laser_map = pcmerge(laser_map, curr_ply, 0.01);
end
figure('NumberTitle', 'off', 'Name', 'demo点云结果与对比路径');
pcshow(laser_map, 'MarkerSize', 20);    %画出demo点云地图
hold on;

%得到demo方法机器人移动轨迹
for i = 1:1:10
    fprintf("第%d处坐标位置:    ",i-10);
    fprintf("%f    ",robot_tf{i}.Translation(1));
    fprintf("%f\n",robot_tf{i}.Translation(2));
end
%得到编程方法机器人移动轨迹
data_road{1}=[0,0,0]';
for k=2:1:10
    data_road{k}=[0,0,0]';
    for j=1:1:k-1
        data_road{k}=Rf_remin{k-j}*data_road{k}+Tf_remin{k-j};
    end
    fprintf('计算第%d处坐标位置：    ',k);
    fprintf('%f,%f',data_road{k}(1),data_road{k}(2));
    fprintf('\n');
end
%画出demo方法机器人移动轨迹
real_location{1}=[0,0,0]';
for i=2:1:10
    real_location{i}=robot_tf{i}.Translation';
    x = [real_location{i-1}(1),real_location{i}(1)];
    y = [real_location{i-1}(2),real_location{i}(2)];
    plot(x,y);
    hold on;
end
%画出编程方法机器人移动轨迹
for i = 2:1:10
    x = [data_road{i}(1),data_road{i-1}(1)];
    y = [data_road{i}(2),data_road{i-1}(2)];
    plot(x,y);
    hold on;
end

%画出10帧叠加点云数据与运动轨迹
str0 = [num2str(10) , '.ply'];
ply_0 = pcread(str0);
p0 = getmat(str0);
data_target=p0';
figure('NumberTitle', 'off', 'Name', '10帧叠加点云数据与运动轨迹');

plot3(0, 0, 0, 'g*');
grid on;
hold on
x=data_target(1,:);
y=data_target(2,:);
plot(x,y,'r.');
hold on
axis equal
for i=2:1:10
    str0 = [num2str(i-1) , '.ply'];
    ply_0 = pcread(str0);
    p0 = getmat(str0);
    data_source=p0';
    k=i;
    for j=1:1:k-1
        data_source=Rf_remin{k-j}*data_source+Tf_remin{k-j};
    end
    x=data_source(1,:);
    y=data_source(2,:);
    plot(x,y,'r.');
    hold on
end
for i = 2:1:10
    x = [real_location{i-1}(1),real_location{i}(1)];
    y = [real_location{i-1}(2),real_location{i}(2)];
    plot(x,y);
    hold on;
end
for i = 2:1:10
    x = [data_road{i}(1),data_road{i-1}(1)];
    y = [data_road{i}(2),data_road{i-1}(2)];
    plot(x,y);
    hold on;
end

axis equal