function [ t ] = tr2t( T )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if numcols(T) ~= numrows(T)
    error('T must be square');
end

n = numcols(T);

if size(T,3) > 1
    t = zeros(size(T,3), 3);
    for i=1:size(T,3)
        t(i,:) = T(1:n-1,n,i)';
    end
else
    t = T(1:n-1,n);
end
end

