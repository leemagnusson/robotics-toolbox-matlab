function [ q ] = GenerateRandomJoints( lower_bound , upper_bound)
%GENERATERANDOMJOINTS Generate random set of joints between lb (lower
%bound) and ub (upper bound)
% Input(s)
%lower_bound: vector of lower bounds
%upper_bound: vector of upper bounds
% 
% Output(s)
%q: vector of random values between upper bound and lower bound

if size(lower_bound) ~= size(upper_bound)
    error('size of the lower and upper bound must match');
elseif ~isvector(lower_bound)
    error('lower bound and upper bound must be vectors');
else
    N = length(lower_bound);
end

q = lower_bound + (upper_bound-lower_bound).*rand(1,N);
end

