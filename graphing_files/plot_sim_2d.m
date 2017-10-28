function [] = plot_sim_2d( )
    path_type = 1;                          % Type 1 = fillet paths
    pWPS = load("output_primary_wps.txt");
    allWPS = load("output_path.txt");
    special_params = load("output_special_path_params.txt");
    allWPS_plus_arc = [];
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
        % Figure out the points to define a fillet path
        path_data = allWPS(wp_index-1:j,:);
        if path_type == 1
            path_data = fillet_path(path_data,special_params(1));
            allWPS_plus_arc = [allWPS_plus_arc;path_data];
        end
        plotPath(path_data);
        wp_index = j +1;
        if length(pWPS(:,1)) > 2
            pause;
        end
    end
    if length(pWPS(:,1)) > 2
        plotTree("output_tree_" + num2str(0) + ".txt", true);
    end
    if path_type == 1
        plotPath(allWPS_plus_arc);
    else
        plotPath(allWPS);
    end
    hold off
end