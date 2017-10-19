function [] = plotTree(filename, flg)
    persistent treeHandle;
    tree = load(filename);
    if isempty(treeHandle)
        treeHandle = plot(tree(:,2),tree(:,1),'Color',[.25 .25 .25]);
    else
        set(treeHandle,'XData',tree(:,2), 'YData',tree(:,1));
    end
    if flg
        set(treeHandle,'XData',[], 'YData',[]);
    end
end