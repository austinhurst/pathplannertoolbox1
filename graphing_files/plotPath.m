function [] = plotPath(filename)
p = load(filename);

figure(1)
hold on
plot(p(:,2),p(:,1),'LineWidth',3,'Color','k');
hold off

end