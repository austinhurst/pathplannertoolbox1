function [ path_data_new ] = fillet_path( path_data,turn_radius )
% Put the fillets into the path
path_data_new = [path_data(1,:)];
if length(path_data(:,1)) > 2
    for i = 2:length(path_data) -1
        par.N = path_data(i-1,1);
        par.E = path_data(i-1,2);
        par.D = path_data(i-1,3);
        mid.N = path_data(i,1);
        mid.E = path_data(i,2);
        mid.D = path_data(i,3);
        nex.N = path_data(i+1,1);
        nex.E = path_data(i+1,2);
        nex.D = path_data(i+1,3);

        a_dot_b = (par.E - mid.E)*(nex.E - mid.E) + (par.N - mid.N)*(nex.N - mid.N) + (par.D - mid.D)*(nex.D - mid.D);
        A = sqrt(pow(par.E - mid.E, 2) + pow(par.N - mid.N, 2) + pow(par.D - mid.D, 2));
        B = sqrt(pow(nex.N - mid.N, 2) + pow(nex.E - mid.E, 2) + pow(nex.D - mid.D, 2));
        Fangle = acos((a_dot_b) / (A*B));
        distance_in = turn_radius / tan(Fangle / 2.0);%// Notice this equation was written incorrectly in the UAV book //sqrt(turn_radius*turn_radius / sin(Fangle / 2.0) / sin(Fangle / 2.0) - turn_radius*turn_radius);
        theta = atan2(nex.N - mid.N, nex.E - mid.E);
        pe.N = (mid.N) + sin(theta)*distance_in;
        pe.E = (mid.E) + cos(theta)*distance_in;
        pe.D = 0;%// 2D, this will need to be fixed once in 3d
        gamma = atan2(par.N - mid.N, par.E - mid.E);
        ps.N = (mid.N) + sin(gamma)*distance_in;
        ps.E = (mid.E) + cos(gamma)*distance_in;
        ps.D = 0;%// 2D, this will need to be fixed once in 3d
        %// Find out whether it is going to the right (cw) or going to the left (ccw)
        %// Use the cross product to see if it is cw or ccw
        cross_product = ((mid.E - ps.E)*(pe.N - mid.N) - (mid.N - ps.N)*(pe.E - mid.E));
        if cross_product < 0
            ccw = false;
        else
            ccw = true;
        end
        if ccw
            cp.N = (mid.N) + sin(gamma - Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
            cp.E = (mid.E) + cos(gamma - Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
            cp.D = mid.D;
            [Nc, Ec] = arc(cp.N,cp.E,turn_radius,gamma + pi/2.0,theta - pi/2.0);
        else
            cp.N = (mid.N) + sin(gamma + Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
            cp.E = (mid.E) + cos(gamma + Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
            cp.D = mid.D;
            if (gamma+3/2.0*pi > pi/2.0)
                gamma = gamma-2*pi;
            end
            sA = theta+pi/2.0;
            eA = gamma + 3/2.0*pi;
            [Nc, Ec] = arc(cp.N,cp.E,turn_radius,theta+pi/2.0,gamma + 3/2.0*pi);
        end
        figure (1)
        %plot(Ec,Nc,'LineWidth',5);
        
        
        Dc = zeros(size(Nc));
        arc_points = [Nc.',Ec.',Dc.'];
        if (ccw == false)
            arc_points = flipud(arc_points);
        end
        path_data_new = [path_data_new;arc_points];
    end
else
    path_data_new = path_data;
end
path_data_new = [path_data_new;path_data(end,:)];
end

function [Nc,Ec] = arc(N, E, r, aS, aE)
    while aE < aS
        aE = aE + 2*pi;
    end
    th = aS:pi/50:aE;
    Ec = r*cos(th)+ E;
    Nc = r*sin(th)+ N;
end
function [out] = pow(a,b)
    out = a^b;
end