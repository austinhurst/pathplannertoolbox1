function [] = plot_sim_3d(varargin)
    path_type = 1;                          % Type 1 = fillet paths
    pWPS = load("output_primary_wps.txt");
    allWPS = load("output_path.txt");
    special_params = load("output_special_path_params.txt");
    allWPS_plus_arc = [];
    f = figure (1);
    hold on
    plotBoundaries3d("output_boundaries.txt",f);
    plotCylinders3d("output_cylinders.txt");
    plotPrimaryWPS3d("output_primary_wps.txt");
    wp_index = 2;
    for i = 1:length(pWPS(:,1))
        for j = wp_index:length(allWPS(:,1))
            if allWPS(j,:) == pWPS(i,:)
                break;
            end
        end
        plotTree3d("output_tree_" + num2str(i-1) + ".txt", false);
        % Figure out the points to define a fillet path
        path_data = allWPS(wp_index-1:j,:);
        if path_type == 1
            path_data = fillet_path(path_data,special_params(1));
            allWPS_plus_arc = [allWPS_plus_arc;path_data];
        end
        plotPath3d(path_data);
        
        total_path_distance = 0;
        for ii = 2:length(path_data(:,1))
            total_path_distance = total_path_distance + sqrt((path_data(ii,1) - path_data(ii-1,1))^2 + (path_data(ii,2) - path_data(ii-1,2))^2 + (path_data(ii,3) - path_data(ii-1,3))^2);
        end
%         disp([i-1, total_path_distance]);
        
        wp_index = j +1;
        if length(pWPS(:,1)) > 2
            pause;
        end
    end
    if length(pWPS(:,1)) > 2
        plotTree3d("output_tree_" + num2str(0) + ".txt", true);
    end
    if path_type == 1
        plotPath3d(allWPS_plus_arc);
        total_path_distance = 0;
        for i = 2:length(allWPS_plus_arc(:,1))
            total_path_distance = total_path_distance + sqrt((allWPS_plus_arc(i,1) - allWPS_plus_arc(i-1,1))^2 + (allWPS_plus_arc(i,2) - allWPS_plus_arc(i-1,2))^2 + (allWPS_plus_arc(i,3) - allWPS_plus_arc(i-1,3))^2);
        end
%         disp([total_path_distance]);
    else
        plotPath3d(allWPS);
        total_path_distance = 0;
        for i = 2:length(allWPS(:,1))
            total_path_distance = total_path_distance + sqrt((allWPS(i,1) - allWPS(i-1,1))^2 + (allWPS(i,2) - allWPS(i-1,2))^2 + (allWPS(i,3) - allWPS(i-1,3))^2);
        end
%         disp([total_path_distance]);
    end
    figure (1)
    hold on
%     all_wps = load('output_path.txt');
%     scatter3(all_wps(:,2),all_wps(:,1),-all_wps(:,3),200,'x','LineWidth',2,'MarkerFaceColor','c','MarkerEdgeColor','c')
    clip = 93000;
    if nargin > 0
        plot3(varargin{1}(1:end -clip ,3), varargin{1}(1:end -clip ,2), -varargin{1}(1:end -clip ,4),'g','LineWidth',5);
    end
    hold off
    
    %% check out the angle, output each waypoint
    allwp = load('output_path.txt');
    nR = length(allwp(:,1));
    nC = length(allwp(1,:));
    final_wangle = zeros(nR,nC+1);
    final_wangle(:,1:end-1) = allwp;
    for i = 1:nR-1
        land_distance = sqrt((final_wangle(i+1,2) - final_wangle(i,2))^2 + (final_wangle(i+1,1) - final_wangle(i,1))^2);
        final_wangle(i,end) = atan2(-final_wangle(i+1,3) + final_wangle(i,3), land_distance)*180.0/3.141592653;
    end
%     disp([final_wangle]);
end