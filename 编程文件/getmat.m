function p = getmat(filename)
% filename = './2.ply';
ptcloud = pcread(filename);
% pcshow(ptcloud);

Data(:,1)= double(ptcloud.Location(1:1:end,1));   %提取所有点的三维坐标
Data(:,2)= double(ptcloud.Location(1:1:end,2));
Data(:,3)= double(ptcloud.Location(1:1:end,3));
namesplit=strsplit(filename,'.ply');           %分割ply文件的名称，分成文件名与ply后缀名
frontname=namesplit{1};                           %提取文件名，舍弃后缀名
%  eval('fid=fopen(''1_buny.txt'',''wt'');');
eval(['fid=fopen(''',frontname,'.txt'',''wt'');']);
[b1,b2]=size(Data);
for i=1:b1                   %将二维数组Data写入txt格式文件中
    for j=1:b2-1
        fprintf(fid,'%.15f\t ',Data(i,j));           %所有坐标数据保留小数点后四位
    end
    fprintf(fid,'%.15f\n',Data(i,b2));
end
clear Data;
fclose(fid);

filename_read =[frontname,'.txt'];%将字符串拼接起来

p = load(filename_read);    %读入txt文件为mat文件