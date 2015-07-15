classdef Link2 < handle
    %LINK2 Generic manipulator link class.
    %   Detailed explanation goes here
    
    properties
        name
        type
        axis
        offset
        qlim
        
        transform
        
        inertia
    end
    
    methods
        function l = Link2(varargin)
            %LINK2 Create robot link object
            if nargin == 0
                % create an 'empty' Link object
                % this call signature is needed to support arrays of Links
                
                %% kinematic parameters
                l.name = '';
                l.type = '';
                l.axis = [0 0 0];
                l.offset = 0;
                l.qlim = [];
                l.transform = eye(4);
                
                %% dynamic parameters
                % these parameters must be set by the user if dynamics is used
                l.inertia = zeros(6);
            elseif nargin == 1 && isa(varargin{1}, 'Link2')
                % clone the passed Link object
                l = copy(varargin{1});
                
            else
                % Create a new Link based on parameters
                
                % parse all possible options
                opt.name = '';
                opt.axis = [0 0 0];
                opt.offset = 0;
                opt.qlim = [];
                opt.type = {'revolute', 'prismatic'};
                opt.transform = zeros(4);
                opt.inertia = zeros(6);
                opt.sym = false;
                
                [opt,args] = tb_optparse(opt, varargin);
                
                if isempty(args)
                    l.name = value( opt.name, opt);
                    l.type = opt.type;
                    l.axis = value( opt.axis, opt);
                    l.offset = value( opt.offset, opt);
                    l.qlim =   value( opt.qlim, opt);
                    
                    l.transform = value( opt.transform, opt);
                    
                    l.inertia = value( opt.inertia, opt);
                end
            end
            
            function out = value(v, opt)
                if opt.sym
                    out = sym(v);
                else
                    out = v;
                end
            end
        end %Link2
        
        function v = isrevolute(L)
            %Link2.isrevolute  Test if joint is revolute
            %
            % L.isrevolute() is true (1) if joint is revolute.
            %
            % See also Link.isprismatic.
            
            v = strcmp(L.type, 'revolute');
        end
        
        function v = isprismatic(L)
            %Link2.isprismatic  Test if joint is prismatic
            %
            % L.isprismatic() is true (1) if joint is prismatic.
            %
            % See also Link.isrevolute.
            v = strcmp(L.type, 'prismatic');
        end
        
        function T = pose(L, q)
            %Link2.pose Link transform matrix
            %
            % T = L.A(Q) is the link homogeneous transformation matrix (4x4) corresponding
            % to the link variable Q which is either rotation angle of
            % revolute joint or translation of prismatic joint
            
            q = q + L.offset;
            
            if strcmp(L.type, 'revolute')            
                ct = cos(q);
                st = sin(q);
                vt = 1 - ct;

                m_vt = vt * L.axis;
                m_st = st * L.axis;
                m_vt_0_1 = m_vt(1)*L.axis(2);
                m_vt_0_2 = m_vt(1)*L.axis(3);
                m_vt_1_2 = m_vt(2)*L.axis(3);

                j_trans = [ct + m_vt(1)*L.axis(1), ...
                           -m_st(3) + m_vt_0_1, ...
                           m_st(2) + m_vt_0_2, ...
                           0; ...
                           m_st(3) + m_vt_0_1, ...
                           ct + m_vt(2)*L.axis(2), ...
                           -m_st(1) + m_vt_1_2, ...
                           0; ...
                           -m_st(2) + m_vt_0_2, ...
                           m_st(1) + m_vt_1_2, ...
                           ct + m_vt(3)*L.axis(3), ...
                           0; 0, 0, 0, 1];
            elseif strcmp(L.type, 'prismatic')
                j_trans = [eye(3), q*L.axis; 0 0 0 1];
            else
                error('RTB:Link2:badtype', 'type must be set to ''revolute'' or ''prismatic'' ');
            end
            T = j_trans * L.transform;
        end
    end
end

