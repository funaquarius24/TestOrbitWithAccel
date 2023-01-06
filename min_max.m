function  min_max(arg)
%MIN_MAX Summary of this function goes here
%   Detailed explanation goes here
sz = size(arg);
if sz(1) > 1 || sz(2) > 1
    min(min(arg))
    max(max(arg))
else
    min(arg)
max(arg)
end


end

