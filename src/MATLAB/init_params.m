%% init_params.m
%
% Description:
%   This function initializes a number of parameters required to simulate
%   rigid-body dynamics.
%
% Inputs:
%   bodies: a cell array of structs, one struct per body, each of which is
%           initialized by calling create_body().
%
% Outputs:
%   params: a struct with many fields

function params = init_params(bodies)
%% General dynamics parameters
params.dyn.grav = 0; % acceleration [m/s^2] due to gravity

%% General visualization parameters
params.viz.margin_scale = 2;

%% Parameters related to the ground:
% RFT parameters:
coeffs = load('RFT_coeffs.mat');
params.terrain.rft.cubic_coefficients = [coeffs.coeffs(1:20)';
                                         coeffs.coeffs(21:40)';
                                         coeffs.coeffs(41:60)'];

params.terrain.scale_factor = 5e3;
params.terrain.surf_fric_coef = 1;

% impact plane geometry parameters:
params.terrain.geom.unit_normal = [0;0;1];
params.terrain.geom.point = zeros(3,1);

% plane visualization parameters:
params.terrain.viz.face.color = 0.5*[0,1,0];
params.terrain.viz.face.alpha = 0.75;
params.terrain.viz.edge.color = 'none';
params.terrain.viz.edge.alpha = 1;

%% Body-specific parameters
params.bodies = bodies;

end