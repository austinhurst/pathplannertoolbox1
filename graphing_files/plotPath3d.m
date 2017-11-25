function [] = plotPath3d(data)
persistent  pathHandle;
if isempty(pathHandle)
    pathHandle = plot3(data(:,2),data(:,1),-data(:,3),'LineWidth',3,'Color','k');
else
    set(pathHandle,'XData',data(:,2),'YData',data(:,1),'ZData',-data(:,3));
end