function [ tau ] = rne( r, a1, a2, a3, a4, a5 )
% TreeLink.rne Inverse dynamics via Recursive Newton-Euler Algorithm
% rne(q,qd,qdd) calculates the inverse dynamics of a kinematic
% tree via the recursive Newton-Euler algorithm.  q, qd and qdd are vectors
% of joint position, velocity and acceleration variables; and the return
% value is a vector of joint force variables.

a_grav = [0; 0; 0; r.gravity];   % default gravity from the object
fext = zeros(6, 1);
n = r.n;

% check that robot object has dynamic parameters for each link
% for j=1:n
%     link = robot.links(j);
%     if isempty(link.r) || isempty(link.I) || isempty(link.m)
%         error('dynamic parameters (m, r, I) not set in link %d', j);
%     end
% end

if numcols(a1) == 3*n
    Q = a1(:,1:n);
    Qd = a1(:,n+1:2*n);
    Qdd = a1(:,2*n+1:3*n);
    np = numrows(Q);
    if nargin >= 3,
        a_grav = [0; 0; 0; a2(:)];
    end
    if nargin == 4
        fext = a3;
    end
else
    np = numrows(a1);
    Q = a1;
    Qd = a2;
    Qdd = a3;
    if numcols(a1) ~= n || numcols(Qd) ~= n || numcols(Qdd) ~= n || ...
            numrows(Qd) ~= np || numrows(Qdd) ~= np
        error('bad data');
    end
    if nargin >= 5,
        a_grav = [0; 0; 0; a4(:)];
    end
    if nargin == 6
        fext = a5;
    end
end

if any([isa(Q,'sym'), isa(Qd,'sym'), isa(Qdd,'sym')])
    tau(np, n) = sym();
else
    tau = zeros(np,n);
end

for p=1:np
    q = Q(p,:).';
    qd = Qd(p,:).';
    qdd = Qdd(p,:).';
    
    for i = 1:n
        tw = r.links(i).twist(1.0);
        S{i} = [tw(4:6), tw(1:3)]';
        vJ = S{i}*qd(i);
        Xup{i} = pluho(inv(r.links(i).pose(q(i))));
        if r.parent(i) == 0
            v{i} = vJ;
            a{i} = Xup{i}*(-a_grav) + S{i}*qdd(i);
        else
            v{i} = Xup{i}*v{r.parent(i)} + vJ;
            a{i} = Xup{i}*a{r.parent(i)} + S{i}*qdd(i) + crm(v{i})*vJ;
        end
        f{i} = r.links(i).spatial_inertia*a{i} + crf(v{i})*r.links(i).spatial_inertia*v{i};
    end
    
    % if nargin == 5
    %   f = apply_external_forces( model.parent, Xup, f, f_ext );
    % end
    
    for i = r.n:-1:1
        tau(p, i) = S{i}' * f{i};
        if r.parent(i) ~= 0
            f{r.parent(i)} = f{r.parent(i)} + Xup{i}'*f{i};
        end
    end
end
end

