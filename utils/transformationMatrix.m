function [A, A_inv] = transformationMatrix(theta, lambda, N, gamma)
%transformationMatrix
% Given a stretch axes and angle of rotation, return a matrix that will
% stretch then rotate (i.e. R2*lambda)
% General option: R2(theta)*lambda*R1(gamma)
%
% TODO: Currently for 2D only.. extend to n dimensional
%
% Inputs:
%   theta       : Rotation angle about z axis [Rad] (R^1)
%   lambda      : Vector to stretch about axes (R^n)
%   N           : Rounding nearest digits to the right (Reduce N for ease
%                   on symbolic computation)
%   gamma       : Rotation angle about z axis [rad] (R^1)
%
% Outputs:
%   A           : Transformation matrix (R^(nxn))
%   A_inv       : A^-1 (Useful for transformation w/ norms) (R^(nxn))

if nargin < 3
    N = 5;
end

if nargin < 4
    gamma = 0; % [rad]
end

R1 = [cos(gamma) -sin(gamma);
        sin(gamma) cos(gamma)];
stretch = lambda;
R2 = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];
A = R2*diag(stretch)*R1;
A_inv = inv(A);

% Round number for function generation
A = round(A, N);
A_inv = round(A_inv, N);

end