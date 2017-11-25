function [] = plotCylinders3d(filename, fnum)
% This file takes in the cylinders from a file (N, E, radius, height) and plots the boundary on figure 1.
    % load in file
    cyls = load(filename);
    falpha = 0.75;
    nps = 50;
    for i = 1:length(cyls(:,1))
        [Nc, Ec] = circle(cyls(i,1), cyls(i,2), cyls(i,3));
        patch(Ec, Nc,ones(size(Nc))+.1,'r','LineStyle','-','FaceAlpha',falpha);
        patch(Ec, Nc,ones(size(Nc))*cyls(i,4),'r','LineStyle','-','FaceAlpha',falpha);
        [X,Y,Z] = cylinder(ones(1,2),31);
        X = X*cyls(i,3) + cyls(i,2);
        Y = Y*cyls(i,3) + cyls(i,1);
        Z = Z*cyls(i,4);
        s = surf(X,Y,Z,'FaceAlpha',falpha,'FaceColor','r');
        s.EdgeColor = 'none';
    end
end

function [Nc,Ec] = circle(N, E, r)
    th = 0:pi/15:2*pi; % 31 points
    Ec = r*cos(th)+ E;
    Nc = r*sin(th)+ N;
end