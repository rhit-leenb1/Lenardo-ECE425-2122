clc
clear all
close all
a = "1 -2 3 -4 5 -6 7 8 -9 10 11 12 13 -14 15 16";
B = str2num(a)
A = reshape(B,4,4)
%A = [1 2 3 4; 5 6 7 8; 9 10 11 12; 13 14 15 16]
A(A==10) = 15
A(A<15) = 0