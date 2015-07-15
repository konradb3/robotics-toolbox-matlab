function [ t, allt ] = fkine( robot, q, tip, varargin )
%FKINE Summary of this function goes here
%   Detailed explanation goes here

opt.base = 0;

opt = tb_optparse(opt, varargin);

if ischar(tip)
    ctip = find(strcmp(robot.names, tip));
else
    ctip = tip;
end

if ischar(opt.base)
    cbase = find(strcmp(robot.names, opt.base));
else
    cbase = opt.base;
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

