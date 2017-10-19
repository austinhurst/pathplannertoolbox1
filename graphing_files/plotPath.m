function [] = plotPath(data)
persistent  pathHandle;
if isempty(pathHandle)
    pathHandle = plot(data(:,2),data(:,1),'LineWidth',3,'Color','k');
else
    set(pathHandle,'XData',data(:,2),'YData',data(:,1));
end