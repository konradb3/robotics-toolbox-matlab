function [ t, allt ] = fkine( robot, q, varargin )
%FKINE Summary of this function goes here
%   Detailed explanation goes here
t = eye(4, 4);

if nargin < 3
    tip = robot.n;
else
    tip = robot.ln2i(varargin{1});
    if isempty(tip)
        vtip = find(strcmp({robot.virtual_links.name}, varargin{1}));
        
        if isempty(vtip)
            error('');
        end
        
        tip = robot.ln2i(robot.virtual_links(vtip).parent);
        t = robot.virtual_links(vtip).origin;
    end
end

if nargin < 4
    base = 0;
else
    base = robot.ln2i(varargin{2});
    if isempty(base)
        error('robot does not contain specyfied base link');
    end
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

