function [] = plot_sim_2d( )
    pWPS = load("output_primary_wps.txt");
    allWPS = load("output_path.txt");
    
    f = figure (1);
    hold on
    plotBoundaries("output_boundaries.txt",f);
    plotCylinders("output_cylinders.txt");
    plotPrimaryWPS("output_primary_wps.txt");
    wp_index = 2;
    for i = 1:length(pWPS(:,1))-1
        for j = wp_index:length(allWPS(:,1))
            if allWPS(j,:) == pWPS(i+1,:)
                break;
            end
        end
        plotTree("output_tree_" + num2str(i-1) + ".txt", false);
        plotPath(allWPS(wp_index-1:j,:));
        wp_index = j +1;
        if length(pWPS(:,1)) > 2
            pause;
        end
    end
    if length(pWPS(:,1)) > 2
        plotTree("output_tree_" + num2str(0) + ".txt", true);
    end
    plotPath(allWPS);
    hold off
end