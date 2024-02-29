function [pe]=DirKin_planar_2DOF(q,L)
% function [pe]=DirKin_planar_2DOF(q,L)
% this function implemnst the direct kinematics of planar robot with 2DOD
% inputs:
%   q=[theta1 theta2] - vector with the joint values (rad)
%   L=[L1 L2] - vector with the two links lengths (mm)
%
%  Aula: 25/10/2023 MAERO
%  Estela Bicho Erlhagen

xe= L(1)*cos(q(1))+L(2)*cos(q(1)+q(2));
ye= L(1)*sin(q(1))+L(2)*sin(q(1)+q(2));

pe =[xe ye]';
end
