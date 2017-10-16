function [ ] = plotTree(filename)
tree = load(filename);
figure (1)
hold on
plot(tree(:,2),tree(:,1),'Color',[.25 .25 .25]);
hold off
end

