%% plot_ground.m
%
% Description:
%   This function plots a flat level surface to represent the ground.
%
% Inputs:
%   ax: a handle to the axes on which to plot the ellipsoid
%   bounds3: a 6-vector [xmin xmax ymin ymax zmin zmax] that specifies the
%        bounds of the plot
%   params: a struct with many fields, including geometry and visualization
%        parameters, generated by calling init_params().
%
% Outputs:
%   none

function plot_ground(ax,bounds3,params)

    x = [bounds3(1),bounds3(2)];
    y = [bounds3(3),bounds3(4)];
    [Xw,Yw] = meshgrid(x,y);
    Zw = zeros(2,2);
    surf(ax,Xw,Yw,Zw,...
        'FaceColor',params.terrain.viz.face.color,...
        'FaceAlpha',params.terrain.viz.face.alpha,...
        'EdgeColor',params.terrain.viz.edge.color,...
        'EdgeAlpha',params.terrain.viz.edge.alpha);

end