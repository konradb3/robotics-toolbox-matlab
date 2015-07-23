function [ t, allt ] = fkine( robot, q, varargin )
%FKINE Summary of this function goes here
%   Detailed explanation goes here

if nargin < 3
    tip = robot.n;
else
    tip = varargin{1};
end

if nargin < 4
    base = 0;
else
    base = varargin{2};
end

if ischar(tip)
    ctip = find(strcmp(robot.names, tip));
else
    ctip = tip;
end

if ischar(base)
    cbase = find(strcmp(robot.names, base));
else
    cbase = base;
end

t = eye(4, 4);

allt = [];

while ctip ~= cbase
    t = robot.links(ctip).pose(q(ctip)) * t;
    allt = [t, allt];
    ctip = robot.parent(ctip);
end

if isa(t, 'sym')
    t = simplify(t);
end

end

