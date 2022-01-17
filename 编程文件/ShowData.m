function ShowData(data1,data2,name)
%SHOWDATA 显示点云数据
    x1 = data1(:, 1);
    y1 = data1(:, 2);
    z1 = data1(:, 3);

    x2 = data2(:, 1);
    y2 = data2(:, 2);
    z2 = data2(:, 3);
    figure('NumberTitle', 'off', 'Name', name);
    %figure();
    plot3(x1, y1, z1, 'r.');
    grid on;
    hold on;
    plot3(x2, y2, z2, 'g.');
    hold off;
    axis equal
end


