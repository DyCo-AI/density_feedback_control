function out1 = rho_f(in1)
%RHO_F
%    OUT1 = RHO_F(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    13-Jan-2023 14:34:33

x1 = in1(1,:);
x2 = in1(2,:);
out1 = 1.0./((cos(x1)+1.0).^2+(cos(x2)-1.0).^2).^(1.0./1.0e+1);
