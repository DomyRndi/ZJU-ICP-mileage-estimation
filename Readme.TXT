0到9.ply文件为180个数据的原始点云文件
10到19.ply文件为剔除最远点后179个数据的点云文件
20到29.ply文件为剔除10个较远数据后170个数据的点云文件
使用nb.bat将后缀.ply更改为.txt
使用nbback.bat将后缀.txt更改为.ply

getmat.m文件可以提取.ply文件中的点云数据变成矩阵
ShowData.m文件将显示点云数据

icp_demo.m文件即为提供的demo文件。
icp_mapping_1.m文件将使用demo提供函数画出原始数据点云地图和移动轨迹
icp_mapping_2.m文件将使用demo提供函数画出179个数据的点云地图和移动轨迹
icp_mapping_3.m文件将使用demo提供函数画出170个数据的点云地图和移动轨迹

icp_an_true_1.m对180个数据求解矩阵R和T，并与demo所得作差得到结果，作出点云地图和运动轨迹
icp_an_true_2.m对179个数据求解矩阵R和T，并与demo所得作差得到结果，作出点云地图和运动轨迹
icp_an_true_3.m对170个数据求解矩阵R和T，并与demo所得作差得到结果，作出点云地图和运动轨迹
