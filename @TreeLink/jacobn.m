%TreeLink.JACOBN Jacobian in end-effector frame
%
% JN = R.jacobn(Q, options) is the Jacobian matrix (6xN) for the robot in
% pose Q, and N is the number of robot joints. The manipulator Jacobian
% matrix maps joint velocity to end-effector spatial velocity V = JN*QD in
% the end-effector frame.
%
% Options::
% 'trans'   Return translational submatrix of Jacobian
% 'rot'     Return rotational submatrix of Jacobian
%
% Notes::
% - This Jacobian is often referred to as the geometric Jacobian.
%
% References::
%  - Differential Kinematic Control Equations for Simple Manipulators,
%    Paul, Shimano, Mayer,
%    IEEE SMC 11(6) 1981,
%    pp. 456-460
%
% See also SerialLink.jacob0, jsingu, delta2tr, tr2delta.




% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
%
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
%
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function J = jacobn(robot, q, varargin)

opt.trans = false;
opt.rot = false;
opt.tip = [];
opt.base = 0;
opt = tb_optparse(opt, varargin);

U = eye(4, 4);

if ~isempty(opt.tip)
    ctip = robot.ln2i(opt.tip);
    if isempty(ctip)
        vtip = find(strcmp({robot.virtual_links.name}, varargin{1}));
        
        if isempty(vtip)
            error('');
        end
        
        ctip = robot.ln2i(robot.virtual_links(vtip).parent);
        U = robot.virtual_links(vtip).origin;
    end
else
    ctip = length(robot.links);
end

if ischar(opt.base)
    cbase = find(strcmp(robot.names, opt.base));
else
    cbase = opt.base;
end

n = robot.n;
L = robot.links;        % get the links

if isa(q, 'sym')
    J(6, robot.n) = sym();
else
    J = zeros(6, robot.n);
end


while ctip ~= cbase
    
    joint_twist = L(ctip).twist(1);

    T_local = L(ctip).pose(q(ctip));
    
    joint_twist = [tr2r(U),  skew(tr2t(U)); zeros(3), tr2r(U)] * joint_twist';
    
    J(:, ctip) = joint_twist;
    
    U = T_local * U;
    
    ctip = robot.parent(ctip);
end

if opt.trans
    J = J(1:3,:);
elseif opt.rot
    J = J(4:6,:);
end

if isa(J, 'sym')
    J = simplify(J);
end
end