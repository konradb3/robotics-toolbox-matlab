function [ R ] = tr2r( T )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    if numcols(T) ~= numrows(T)
        error('T must be square');
    end

    n = numcols(T);

    if size(T,3) > 1
        R = zeros(3,3,size(T,3));
        for i=1:size(T,3)
            R(:,:,i) = T(1:n-1,1:n-1,i);
        end
    else
        R = T(1:n-1,1:n-1);
    end

end

