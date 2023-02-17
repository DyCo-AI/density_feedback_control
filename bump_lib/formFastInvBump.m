function inv_bump = formFastInvBump(x, params, N, A, vpa_enable)
%formFastInvBump
% Forms fast inverse bump function. The following bump function is
% representative of the bump function in paper; however, following 
% assumptions are made s.t. the formation of the bump function is
% fast:
%   1) Zero level set must not overlap (due to how the piecewise is formed,
%       the following bump function formulation is not representative of
%       how the effects of the true bump function (in paper) will result
%       when overlapping
%       i.e. When zero level set is overlapping, this does not preserve
%       the operation A U B, moreso, B overrides A or vice versa at the
%       intersection
%
% Inputs:
%   x           : Real Symbolic variable (R^(nx1))
%   params      : Structure parameter containing r1, r2, and the centers
%                   of the obstacles (Centers of obstacle must have the
%                   following variable name, ci, where i is an positive
%                   integer (i >=1)
%   N           : Number of obstacles (R^1) w/ centers (c1-cN)
%   A           : Transformation matrix to rotate and stretch the obstacle
%   vpa_enable  : Boolean to enable variable prevision arithmetic
%
% Outputs:
%   bump        : Single-valued symbolic function of a inverse bump that
%                   has the following centers (ci) with radii (r1 & r2)

%TODO: Bug, where when using rotation and 4 obstacles -> 1st obstacle
%disappears. Figure out why

if nargin < 4
    A = diag([1,1]);
end

if nargin < 5
    vpa_enable = false;
end



r1_list = params.r1;
r2_list = params.r2;

if length(r1_list) == 1 && length(r2_list) == 1
    r1 = r1_list;
    r2 = r2_list;
end

p = params.p;

for i = 1:N
    if length(r1_list) > 1
        r1 = r1_list(i);
        r2 = r2_list(i);
    end
        
    c = params.(sprintf("c%d", i));
    
    m = (norm(A*(x-c), p)^p - r1^p)/(r2^p-r1^p);
    f = piecewise(m<=0, 0, exp(-1/m));
    f_shift = piecewise(1-m<=0, 0,  exp(-1/(1-m)));
    g = f/(f+f_shift);
    if vpa_enable
        g = vpa(g,2);
    end
    if i > 1
        h = piecewise(norm(A*(x-c), p)^p >= r2^p, h_prev, g); 
    else
        h = g;
        % h = piecewise(norm(A*(x-c), p)^p >= r2^p, 1, g); 
    end
        
    h_prev = h;
end

inv_bump = h;

end