function [ T ] = T( v0 )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

T=eye(4);
T(1:3,4)=v0;
end

