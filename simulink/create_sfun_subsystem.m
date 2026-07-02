function create_sfun_subsystem(sfun_dir, func_name, n_in, n_out, in_dims, out_dims)
%% create_sfun_subsystem.m — 通用Level-2 MATLAB S-Function生成器
% 为任意动力学函数生成纯代数S-Function包装（无内部状态）
% 配合Simulink积分器使用——所有状态由Simulink积分器管理
%
% 输入:
%   sfun_dir  — S-Function输出目录
%   func_name — 被包装的MATLAB函数名（如 'suspension'）
%   n_in      — 输入端口数量
%   n_out     — 输出端口数量
%   in_dims   — 各输入端口的维度
%   out_dims  — 各输出端口的维度

sfun_name = [func_name, '_sfun'];

%% 使用cell逐步构建——全部使用end+1追加，不使用{...}字面量嵌入for
sfun = {};  % 代码行cell

% 函数头
sfun{end+1} = ['function ', sfun_name, '(block)'];
sfun{end+1} = ['%% Level-2 MATLAB S-Function wrapper for ', func_name, '.m'];
sfun{end+1} = '% 纯代数计算——状态由外部Simulink积分器管理';
sfun{end+1} = '';
sfun{end+1} = 'setup(block);';
sfun{end+1} = '';

% setup 函数
sfun{end+1} = 'function setup(block)';
sfun{end+1} = ['    block.NumInputPorts  = ', num2str(n_in), ';'];

% 输入端口维度
for i = 1:n_in
    sfun{end+1} = sprintf('    block.InputPort(%d).Dimensions = %d;', i, in_dims(i));
end
% 所有输入端口均为直接馈通（纯代数块，输出取决于当前输入）
for i = 1:n_in
    sfun{end+1} = sprintf('    block.InputPort(%d).DirectFeedthrough = true;', i);
end
sfun{end+1} = '    ';

% 输出端口
sfun{end+1} = ['    block.NumOutputPorts = ', num2str(n_out), ';'];
for i = 1:n_out
    sfun{end+1} = sprintf('    block.OutputPort(%d).Dimensions = %d;', i, out_dims(i));
end
sfun{end+1} = '    ';

% 参数和设置
sfun{end+1} = '    block.NumDialogPrms = 1;';
sfun{end+1} = '    block.SampleTimes = [0 0];';
sfun{end+1} = '    ';
sfun{end+1} = '    block.RegBlockMethod(''Outputs'', @Outputs);';
sfun{end+1} = '    block.RegBlockMethod(''SetInputPortSamplingMode'', @SetInpPortFrameData);';
sfun{end+1} = 'end';
sfun{end+1} = '';

% SetInpPortFrameData
sfun{end+1} = 'function SetInpPortFrameData(block, idx, fd)';
sfun{end+1} = '    block.InputPort(idx).SamplingMode = fd;';
for i = 1:n_in
    sfun{end+1} = sprintf('    block.InputPort(%d).SamplingMode = fd;', i);
end
for i = 1:n_out
    sfun{end+1} = sprintf('    block.OutputPort(%d).SamplingMode = fd;', i);
end
sfun{end+1} = 'end';
sfun{end+1} = '';

% Outputs 函数
sfun{end+1} = 'function Outputs(block)';

% 读取所有输入到cell数组
for i = 1:n_in
    sfun{end+1} = sprintf('    u{%d} = block.InputPort(%d).Data;', i, i);
end
sfun{end+1} = '    p = block.DialogPrm(1).Data;';
sfun{end+1} = '    ';

% 构建函数调用参数列表
call_args = '';
for i = 1:n_in
    if i > 1, call_args = [call_args, ', ']; end
    call_args = [call_args, sprintf('u{%d}', i)];
end
call_args = [call_args, ', p'];

% 构建输出变量列表
out_vars = '';
for i = 1:n_out
    if i > 1, out_vars = [out_vars, ', ']; end
    out_vars = [out_vars, sprintf('o%d', i)];
end

% 函数调用
if n_out == 1
    sfun{end+1} = sprintf('    o1 = %s(%s);', func_name, call_args);
else
    sfun{end+1} = sprintf('    [%s] = %s(%s);', out_vars, func_name, call_args);
end
sfun{end+1} = '    ';

% 输出赋值
for i = 1:n_out
    sfun{end+1} = sprintf('    block.OutputPort(%d).Data = o%d;', i, i);
end
sfun{end+1} = 'end';

% 外层函数end（MATLAB要求：嵌套函数使用end时，外层也必须用end）
sfun{end+1} = 'end';

%% 写文件
sfun_path = fullfile(sfun_dir, [sfun_name, '.m']);
fid = fopen(sfun_path, 'w');
for k = 1:length(sfun)
    fprintf(fid, '%s\n', sfun{k});
end
fclose(fid);
fprintf('  生成 S-Function: %s (%d入 → %d出)\n', sfun_name, n_in, n_out);
end
