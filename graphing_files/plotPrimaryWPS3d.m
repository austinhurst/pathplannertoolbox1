function [] = plotPrimaryWPS3d(filename, fnum)
% This file takes in the cylinders from a file (N, E, radius, height) and plots the boundary on figure 1.
    % load in file
    wps = load(filename);
    scatter3(wps(:,2),wps(:,1), -wps(:,3),200,'x','LineWidth',2,'MarkerFaceColor','b','MarkerEdgeColor','b');
    a = [1:length(wps(:,1))]'; b = num2str(a); c = cellstr(b);
    dx =30; dy = 30; dz = 10;% displacement so the text does not overlay the data points
    text(wps(:,2)+dx, wps(:,1)+dy, -wps(:,3)+dz, c);
end