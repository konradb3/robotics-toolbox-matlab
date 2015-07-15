classdef TreeLink < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        name
        manuf
        comment
    end
    
    properties (SetAccess = private)
        n
        links
        parent
    end
    
    properties (Dependent = true)
        offset
        qlim
        names
    end
    
    methods
        function r = TreeLink(L, varargin)
            r.name = 'noname';
            r.manuf = '';
            r.comment = '';
            r.links = [];
            r.parent = [];
            r.n = 0;
            
            % process the rest of the arguments in key, value pairs
            opt.name = [];
            opt.comment = [];
            opt.manufacturer = [];
            opt.offset = [];
            opt.qlim = [];
            opt.parent = [];

            [opt,out] = tb_optparse(opt, varargin);
            if ~isempty(out)
                error('unknown option <%s>', out{1});
            end
            
            if nargin == 0
                % zero argument constructor, sets default values
                return;
            else
                if isa(L, 'Link2')
                    r.links = L;    % attach the links
                else
                    error('unknown type passed to robot');
                end
                r.n = length(r.links);
            end
            
            
            % copy the properties to robot object
            p = properties(r);
            for i=1:length(p)
                if isfield(opt, p{i}) && ~isempty(getfield(opt, p{i}))
                    r.(p{i}) = getfield(opt, p{i});
                end
            end
            
            if isempty(r.parent)
                r.parent = 0:(r.n-1);
            elseif length(r.parent) ~= length(r.links)
                error('parent size does not match links size')
            end
        end
        
        function set.offset(r, v)
            if length(v) ~= length(v)
                error('offset vector length must equal number DOF');
            end
            L = r.links;
            for i=1:r.n
                L(i).offset = v(i);
            end
            r.links = L;
        end

        function v = get.offset(r)
            v = [r.links.offset];
        end

        function set.qlim(r, v)
            if numrows(v) ~= r.n
                error('insufficient rows in joint limit matrix');
            end
            L = r.links;
            for i=1:r.n
                L(i).qlim = v(i,:);
            end
            r.links = L;
        end

        function v = get.qlim(r)
            L = r.links;
            v = zeros(r.n, 2);
            for i=1:r.n
                if isempty(L(i).qlim)
                    if L(i).isrevolute
                        v(i,:) = [-pi pi];
                    else
                        v(i,:) = [-Inf Inf];
                    end
                else
                    v(i,:) = L(i).qlim;
                end
            end
        end
        
        function set.names(r, v)
            if length(v) ~= length(v)
                error('names vector length must equal number DOF');
            end
            L = r.links;
            for i=1:r.n
                L(i).name = v(i);
            end
            r.links = L;
        end

        function v = get.names(r)
            v = {r.links.name};
        end
    end
    
end

