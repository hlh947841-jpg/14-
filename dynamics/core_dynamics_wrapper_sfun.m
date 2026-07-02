function core_dynamics_wrapper_sfun(block)
%% Level-2 MATLAB S-Function wrapper for core_dynamics_wrapper.m
% 纯代数计算——状态由外部Simulink积分器管理

setup(block);

function setup(block)
    block.NumInputPorts  = 3;
    block.InputPort(1).Dimensions = 1;
    block.InputPort(2).Dimensions = 29;
    block.InputPort(3).Dimensions = 11;
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(2).DirectFeedthrough = true;
    block.InputPort(3).DirectFeedthrough = true;
    
    block.NumOutputPorts = 2;
    block.OutputPort(1).Dimensions = 29;
    block.OutputPort(2).Dimensions = 52;
    
    block.NumDialogPrms = 1;
    block.SampleTimes = [0 0];
    
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
end

function SetInpPortFrameData(block, idx, fd)
    block.InputPort(idx).SamplingMode = fd;
    block.InputPort(1).SamplingMode = fd;
    block.InputPort(2).SamplingMode = fd;
    block.InputPort(3).SamplingMode = fd;
    block.OutputPort(1).SamplingMode = fd;
    block.OutputPort(2).SamplingMode = fd;
end

function Outputs(block)
    u{1} = block.InputPort(1).Data;
    u{2} = block.InputPort(2).Data;
    u{3} = block.InputPort(3).Data;
    p = block.DialogPrm(1).Data;
    
    [o1, o2] = core_dynamics_wrapper(u{1}, u{2}, u{3}, p);
    
    block.OutputPort(1).Data = o1;
    block.OutputPort(2).Data = o2;
end
end
