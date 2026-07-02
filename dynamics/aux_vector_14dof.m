function [v, names] = aux_vector_14dof(aux)
%% aux_vector_14dof.m — 将 vehicle_dynamics 诊断结构打包为固定长度向量
% 用于 Simulink S-Function 输出和 To Workspace 保存，避免结构体信号导致模型接口不稳定。

Fz_left = aux.Fz_tire(1) + aux.Fz_tire(3);
Fz_right = aux.Fz_tire(2) + aux.Fz_tire(4);
LTR = (Fz_left - Fz_right) / max(Fz_left + Fz_right, 1e-3);

v = [
    aux.delta_eff(:)
    aux.i_h_eff
    aux.i_c_eff
    aux.Fz_mf(:)
    aux.Fz_tire(:)
    aux.Fs(:)
    aux.alpha(:)
    aux.kappa(:)
    aux.Fx(:)
    aux.Fy(:)
    aux.T_drive(:)
    aux.T_brake(:)
    aux.T_m(:)
    aux.beta_brk
    aux.beta_brk_auto
    LTR
    aux.ax_body
    aux.ay_body
    aux.Mz
];

if nargout > 1
    wheel_names = {'FL','FR','RL','RR'};
    names = {};
    names = append_wheel_names(names, 'delta_eff', wheel_names);
    names = [names, {'i_h_eff','i_c_eff'}];
    names = append_wheel_names(names, 'Fz_mf', wheel_names);
    names = append_wheel_names(names, 'Fz_tire', wheel_names);
    names = append_wheel_names(names, 'Fs', wheel_names);
    names = append_wheel_names(names, 'alpha', wheel_names);
    names = append_wheel_names(names, 'kappa', wheel_names);
    names = append_wheel_names(names, 'Fx', wheel_names);
    names = append_wheel_names(names, 'Fy', wheel_names);
    names = append_wheel_names(names, 'T_drive', wheel_names);
    names = append_wheel_names(names, 'T_brake', wheel_names);
    names = append_wheel_names(names, 'T_m', wheel_names);
    names = [names, {'beta_brk','beta_brk_auto','LTR','ax_body','ay_body','Mz'}];
end
end

function names = append_wheel_names(names, prefix, wheel_names)
for i = 1:numel(wheel_names)
    names{end + 1} = [prefix, '_', wheel_names{i}]; %#ok<AGROW>
end
end
