function [ t, allt ] = fkine( robot, q, varargin )
%FKINE Summary of this function goes here
%   Detailed explanation goes here
t = eye(4, 4);


opt.tip = [];
opt.base = [];
opt = tb_optparse(opt, varargin);

if isempty(opt.tip)
    tip = robot.n;
else
    tip = robot.ln2i(opt.tip);
    if isempty(tip)
        vtip = find(strcmp({robot.virtual_links.name}, opt.tip));
        
        if isempty(vtip)
            error('');
        end
        
        tip = robot.ln2i(robot.virtual_links(vtip).parent);
        t = robot.virtual_links(vtip).origin;
    end
end

if isempty(opt.base)
    base = 0;
else
    base = robot.ln2i(opt.base);
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

if nargout > 1
    allt = zeros(4,4,robot.n);
    if isa(q,'sym')
        allt = sym(allt);
    end
end

while ctip ~= cbase
    t = robot.links(ctip).pose(q(ctip)) * t;
    ctip = robot.parent(ctip);
end

if nargout > 1
    parfor i = 1:robot.n
        allt(:,:,i) = robot.fkine(q, i, cbase); % intermediate transformations
    end
end

if isa(t, 'sym')
    t = simplify(t);
end

end

