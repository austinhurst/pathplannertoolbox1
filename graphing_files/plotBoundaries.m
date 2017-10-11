function [ ] = plotBoundaries( filename )
% This file takes in the boundaries from a file (just the points in North,
% Down) and plots the boundary on figure 1.

% load in file
pts = load(filename);
pts = [pts; pts(1,:)];

% calculate the figure limits
maxN = max(pts(:,1));
minN = min(pts(:,1));
Nspan = maxN - minN;
maxE = max(pts(:,2));
minE = min(pts(:,2));
Espan = maxE - minE;
span = max(Nspan,Espan)*1.05;
lims = [(maxE + minE)/2 - span/2, (maxE + minE)/2 + span/2; ...
    (maxN + minN)/2 - span/2, (maxN + minN)/2 + span/2];


f = figure (1);
hold on;
% Set the figure sizes
xlim(lims(1,:));
ylim(lims(2,:));
axis square
set(f,'Position',[743 79 616 582]);

% color in the background
fill([lims(1,1), lims(1,2), lims(1,2), lims(1,1)],...
    [lims(2,1), lims(2,1), lims(2,2), lims(2,2)],[.7 .7 .7]);

% 'WHITE OUT' THE COMPETITION AREA
fill(pts(:,2),pts(:,1),'w');

% Labels
xlabel('East (m)');
ylabel('North (m)');
set(gca,'FontSize',12);

end

