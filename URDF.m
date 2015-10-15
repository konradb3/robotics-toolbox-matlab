classdef URDF
    
    properties
        name
        joints
        links
        transmissions
        base_link_name
    end
    
    methods
        function urdf = URDF(urdffile)
            if nargin == 0
                urdffile = 'katana_400_6m180.urdf.xacro';
            end
            
            dom = xmlread(urdffile);
            
            robot = dom.getElementsByTagName('robot').item(0);
            
            urdf.name = URDF.getAttributeValue(robot, 'name');
            
            elements = robot.getChildNodes();
            
            joint_idx = 1;
            link_idx = 1;
            
            for i = 1:elements.getLength
                element = elements.item(i-1);
                
                switch char(element.getNodeName())
                    case 'joint'
                        j = URDF.parseJoint(element);
                        urdf.joints{joint_idx} = j;
                        joint_idx = joint_idx + 1;
                    case 'link'
                        l = URDF.parseLink(element);
                        urdf.links{link_idx} = l;
                        link_idx = link_idx + 1;
                end
            end
            
            p = zeros(1, length(urdf.links));
            
            for j=1:length(urdf.joints)
                if ~isfield(urdf.joints{j}, 'parent')
                    error('joint without parrent [%s]', urdf.joints{j}.name);
                end
                i = urdf.ln2i( urdf.joints{j}.child);
                p(i) = p(i) - 1;
            end
            
            
            [~,base_link] = max(p);
            urdf.base_link_name = urdf.links{base_link}.name;
        end
        
        function r = robot(urdf, varargin)
            
            % process the rest of the arguments in key, value pairs
            opt.base = urdf.base_link_name;
            opt.blacklist = [];
            opt.symbolic = false;

            [opt,out] = tb_optparse(opt, varargin);
            if ~isempty(out)
                error('unknown option <%s>', out{1});
            end
            
            r = TreeLink();
            r.name = urdf.name;
            base_name = opt.base;
            [l, vl] = urdf.makelinks(base_name, opt.blacklist, opt.symbolic);
            
            for link = l
                if strcmp(link.parent, base_name)
                    parent = 0;
                else
                    parent = link.parent;
                end
                r.addLink(Link2('name', link.name, 'type', link.type, 'axis', link.axis, 'transform', link.transform), parent);
            end
            
            for vlink = vl
                r.addVirtualLink(vlink.name, vlink.parent, vlink.transform);
            end
            
        end
        
        function [l, vl] = makelinks(urdf, name, blacklist, symbolic)
            l = [];
            vl = [];
            j = urdf.findnextjoint(name);
            for i = j
                joint = urdf.joints{i};
                
                if ~isempty(find(strcmp(blacklist, joint.child), 1))
                    continue;
                end
                
                if strcmp(joint.type, 'fixed')
                    cvl.parent = name;
                    cvl.name = joint.child;
                    cvl.transform = urdf.make_transform(joint.origin, symbolic);
                    
                    [ll, vvl] = urdf.makelinks(joint.child, blacklist, symbolic);
                    
                    for n = 1:length(ll)
                        if strcmp(ll(n).parent, joint.child)
                          ll(n).parent = name;
                          ll(n).transform = urdf.make_transform(joint.origin, symbolic) * ll(n).transform;
                        end
                    end
                    
                    for n = 1:length(vvl)
                        if strcmp(vvl(n).parent, joint.child)
                          vvl(n).parent = name;
                          vvl(n).transform = urdf.make_transform(joint.origin, symbolic) * vvl(n).transform;
                        end
                    end
                    
                    l = [l, ll];
                    vl = [vl, cvl, vvl];
                    
                else
                    link.name = joint.child;
                    link.type = joint.type;
                    if symbolic
                        link.axis =  sym(joint.axis);
                    else
                        link.axis =  joint.axis;
                    end
                    link.transform = urdf.make_transform(joint.origin, symbolic);
                    link.parent = name;
                    [ll, vvl] = urdf.makelinks(joint.child, blacklist, symbolic);
                    l = [l, link, ll];
                    vl = [vl, vvl];
                end
            end
        end
        
        function n = nlinks(urdf)
            n = length(urdf.links);
        end
        
        function n = njoints(urdf)
            n = length(urdf.joints);
        end
        
        % map joint/link name to an index
        
        function joint = findnextjoint(urdf, link)
            joint = find(cellfun(@(x)strcmp(x.parent,link),urdf.joints));
        end
        
        function idx = ln2i(urdf, name)
            idx = [];
            for i=1:length(urdf.links)
                if strcmp(urdf.links{i}.name, name)
                    idx = i;
                    return
                end
            end
        end
        
%         function display(urdf)
%             for j=1:length(urdf.joints)
%                 joint = urdf.joints{j};
%                 fprintf('j%d: %s -> %s (%s)\n', j, joint.parent.link, joint.child.link, joint.type);
%             end
%         end
    end
    
    methods (Static)
        function t = make_transform(tr, symbolic)
            if ~isempty(tr.quat)
                if symbolic
                    quat = sym(tr.quat);
                else
                    quat = tr.quat;
                end
                q = Quaternion(quat);
                rot = q.R;
            else
                if symbolic
                    rot = rpy2r(sym('pi')./sym(round(pi./tr.rpy, 10)));
                else
                    rot = rpy2r(pi./round(pi./tr.rpy, 10));
                end
            end
            
            t = rt2tr(rot, tr.xyz(:));
        end
        
        function joint = parseJoint(node)
            val = URDF.getAttributeValue(node, 'name');
            if isempty(val)
                error('Joint without name');
            end
            joint.name = val;
            
            val = URDF.getAttributeValue(node, 'type');
            if isempty(val)
                error('Joint without type');
            end
            joint.type = val;
            
            node_children = node.getChildNodes;
            for i=1:node_children.getLength
                child = node_children.item(i-1);
                switch char(child.getNodeName)
                    case 'parent'
                        p = URDF.getAttributeValue(child, 'link');
                        if isempty(p)
                            error('joint:parent must have link attribute');
                        end
                        joint.parent = p;
                    case 'child'
                        c = URDF.getAttributeValue(child, 'link');
                        if isempty(c)
                            error('joint:child must have link attribute');
                        end
                        joint.child = c;
                    case 'axis'
                        a = URDF.getAttributeValue(child, 'xyz');
                        if isempty(a)
                            error('joint:child must have xyz attribute');
                        end
                        joint.axis = a;
                    case 'origin'
                        xyz = URDF.getAttributeValue(child, 'xyz');
                        if isempty(xyz)
                            xyz = [0 0 0];
                        end
                        joint.origin.xyz = xyz;
                        
                        rpy = URDF.getAttributeValue(child, 'rpy');
                        if ~isempty(rpy)
                            joint.origin.rpy = rpy;
                        end
                        
                        quat = URDF.getAttributeValue(child, 'quaternion');
                        if ~isempty(rpy)
                            joint.origin.quat = quat;
                        end
                end
            end
            
        end
        
        function link = parseLink(node)
            val = URDF.getAttributeValue(node, 'name');
            if isempty(val)
                error('Link without name');
            end
            link.name = val;
        end
        
        function val = getAttributeValue(node, name)
            attributes = node.getAttributes;
            att = attributes.getNamedItem(name);
            
            if isempty(att)
                val = [];
            else
                v = char(att.getValue());
                val = str2num(v);
                if isempty( val )
                    val = v;
                end
            end
        end
    end
end

