function [] = plotTree3d(filename, flg)
    persistent treeHandle;
    tree = load(filename);
    if isempty(treeHandle)
        treeHandle = plot3(tree(:,2),tree(:,1),-tree(:,3),'Color',[.25 .25 .25]);
    else
        set(treeHandle,'XData',tree(:,2), 'YData',tree(:,1),'ZData',-tree(:,3));
    end
    if flg
        set(treeHandle,'XData',[], 'YData',[],'ZData',[]);
    end
end