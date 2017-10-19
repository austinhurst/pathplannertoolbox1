function [] = plotPrimaryWPS(filename, fnum)
% This file takes in the cylinders from a file (N, E, radius, height) and plots the boundary on figure 1.
    % load in file
    wps = load(filename);
    for i = 1:length(wps(:,1))
        scatter(wps(:,2),wps(:,1),200,'x','LineWidth',2);
    end
    a = [1:length(wps(:,1))]'; b = num2str(a); c = cellstr(b);
    dx =30; dy = 30; % displacement so the text does not overlay the data points
    text(wps(:,2)+dx, wps(:,1)+dy, c);
end