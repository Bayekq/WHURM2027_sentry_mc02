% 指定文件夹路径
folderPath = './';

% 获取所有CSV文件
files = dir(fullfile(folderPath, '*.csv'));

% 初始化存储表格的单元格数组
tablesCell = cell(length(files), 1);

% 遍历处理每个文件
for i = 1:length(files)
    filePath = fullfile(files(i).folder, files(i).name);
    
    % 读取表格
    tempTable = readtable(filePath);
    
    % 提取并重命名指定列
    extractedTable = tempTable(:, {'I10', 'I11', 'I12'});
    extractedTable.Properties.VariableNames = {'accel_x', 'accel_y', 'accel_z'};
    
    tablesCell{i} = extractedTable;
end

% 合并所有表格
mergedTable = vertcat(tablesCell{:});

% 保存结果
writetable(mergedTable, 'accel_calib.csv');

fprintf('处理完成！合并了 %d 个文件，总行数: %d\n', length(files), height(mergedTable));