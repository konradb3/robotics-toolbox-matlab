function [ L ] = DHLink( varargin )
%DHLINK Create Link2 object for given DH parameters
%   Detailed explanation goes here
% parse all possible options
opt.theta = [];
opt.a = 0;
opt.d = [];
opt.alpha = 0;
opt.type = {'revolute', 'prismatic', 'fixed'};
opt.convention = {'standard', 'modified'};


[opt,args] = tb_optparse(opt, varargin);

if ~isempty(opt.theta)
    theta = opt.theta;
else
    theta = 0;
end

if ~isempty(opt.d)
    d = opt.d;
else
    d = 0;
end

st = sin(theta);
ct = cos(theta);
sa = sin(opt.alpha);
ca = cos(opt.alpha);

if strcmp(opt.convention, 'standard')
    dh_tr = rt2tr([
        ct, -st*ca,  st*sa;
        st,  ct*ca, -ct*sa;
        0,      sa,      ca], ...
        [opt.a*ct; opt.a*st; d]);
elseif stdcmp(opt.convention, 'modified')
    dh_tr = rt2tr([
        ct,       -st,     0;
        st*ca,  ct*ca,   -sa;
        st*sa,  ct*sa,    ca   ], ...
        [opt.a;      -sa*d;  ca*d ]);
end

if ~isempty(opt.theta)
    % constant value of theta means it must be prismatic
    L = Link2('type', 'prismatic', 'axis', [0 0 1], 'transform', dh_tr, args{:});
end

if ~isempty(opt.d)
    % constant value of d means it must be revolute
    L = Link2('type', 'revolute', 'axis', [0 0 1], 'transform', dh_tr, args{:});
end
if ~isempty(opt.d) && ~isempty(opt.theta)
    error('RTB:DHLink:badarg', 'specify only one of ''d'' or ''theta''');
end
end

