%% main.m
%
% Description:
%   Program entry point. Calls various functions.
%
% Dependencies:
%   Modern Robotics
%
% Inputs:
%   none
%
% Outputs:
%   none

function main
%% Initialization
clear;
close all;
clc;

init_env;

%% Initialize each body (geometry/visuals and state)

num_bodies = 3;

% initialize geometric and visual properties for each body:
primitives = {'rect-prism','cylinder','ellipsoid'};
friendly_names = {'Moe','Larry','Curly'};
semi_axes13.x = 0.1;
semi_axes13.y = 0.15;
semi_axes13.z = 0.2;
semi_axes2.r = 0.15;
semi_axes2.h = 0.2;

semi_axes = {semi_axes13,semi_axes2,semi_axes13};

for n = 1:num_bodies
    mass = n;
    body_params{n} = create_body(friendly_names{n},...
                                 primitives{n},...
                                 semi_axes{n},...
                                 mass);
end

params = init_params(body_params);

% initialize configuration and twist ("state") for each body:
Twb_init = zeros(4*num_bodies,4);
Vb_init = zeros(6*num_bodies,1);

x_init = [-1,0,1];
y_init = [0,0,0];
z_init = [1,1,1];
for n = 1:num_bodies
    Twb_row_start = 1 + (n-1)*4;
    Twb_row_end = Twb_row_start + 3;
    Twb_n{n}(1:3,1:3) = eye(3); % orientation of body n in {w}
    Twb_n{n}(1:3,4) = [x_init(n);
                       y_init(n);
                       z_init(n)]; % position of body n CoM in {w}
    Twb_n{n}(4,:) = [0,0,0,1]; % bottom row of 4x4 homog. transf. matrix
    Twb_init(Twb_row_start:Twb_row_end,:) = Twb_n{n};
    
    Vb_row_start = 1 + (n-1)*6;
    Vb_row_end = Vb_row_start + 5;
    Vb_init(Vb_row_start + 1) = 10;
    Vb_init(Vb_row_start) = 1e-3;
end

%% Set simulation parameters
dt = 1e-2; % timestep size [s]
t0 = 0;
tf = 20; % simulation duration [s]
t = t0:dt:tf;

% event location parameters:
bis_err_tol = 1e-6;
bis_its_max = 100;

ode_fun = @(t,T,V) rbdyn(t,T,V,params);
event_fun = @(T) dist_to_impact(T,params);

%% Simulate via numerical integration
Twb{1} = Twb_init;
Vb{1} = Vb_init;
tic;
for k = 1:numel(t)-1
%     fprintf('t = %5.3f s.\n',t(k))
    [~,Twb{k+1},Vb{k+1}] = cg4(ode_fun,dt,t(k),Twb{k},Vb{k},params);
    
%     [t_e,Twb_e,Vb_e] = event_locator(event_fun,...
%                                      ode_fun,...
%                                      t(k),t(k+1),...
%                                      Twb{k},Twb{k+1},...
%                                      Vb{k},Vb{k+1},...
%                                      bis_err_tol,bis_its_max);
                        
%     if ~isempty(t_e)
%         fprintf('\tEvent at t = %f s\n',t_e);
%     end
end
toc;

%% Animate simulation
options.trace_CoM = true;
options.record_video = true;
options.video_name = 'dzhanibekov_three-bodies.avi';
animate(Twb,t,params,options);

%% Postprocess
for n = 1:num_bodies
    Twb_row_start = 1 + (n-1)*4;
    Twb_row_end = Twb_row_start + 3;
    
    Vb_row_start = 1 + (n-1)*6;
    Vb_row_end = Vb_row_start + 5;
    
    % kinematics:
    Twb_hist{n} = NaN(4,4,numel(t));
    Vb_hist{n} = NaN(6,numel(t));
    p_hist{n} = NaN(3,numel(t));
    v_hist{n} = NaN(3,numel(t));

    Gb = params.bodies{n}.dyn.inertia_matrix_6D;

    % energy and momenta:
    E_hist{n} = NaN(1,numel(t)); % mechanical energy
    Lb_hist{n} = NaN(3,numel(t)); % angular momentum in body frame
    Pb_hist{n} = NaN(3,numel(t)); % linear momentum in body frame

    for k = 1:numel(t)
        % orientation of {b} in {w}:
        Rwb = Twb{k}(Twb_row_start:Twb_row_end-1,1:3);
        % position of origin of {b} in {s}:
        pwb = Twb{k}(Twb_row_start:Twb_row_end-1,4);

        % velocity of CoM in world frame:
        v = Rwb*Vb{k}(4:6);

        % momentum:
        Pb = Gb*Vb{k}(Vb_row_start:Vb_row_end);

        % energy
        KE = 0.5*transpose(Vb{k}(Vb_row_start:Vb_row_end))...
           * Gb*Vb{k}(Vb_row_start:Vb_row_end);
        PE = params.bodies{n}.dyn.mass*params.dyn.grav*pwb(3);
        E = KE + PE;

        Twb_hist{n}(:,:,k) = Twb{k}(Twb_row_start:Twb_row_end,:);
        Vb_hist{n}(:,k) = Vb{k}(Vb_row_start:Vb_row_end);
        p_hist{n}(:,k) = pwb;
        v_hist{n}(:,k) = v;
        Lb_hist{n}(:,k) = [eye(3),zeros(3)]*Pb; % angular momentum in {b}
        Pb_hist{n}(:,k) = [zeros(3),eye(3)]*Pb; % linear momentum in {b}
        E_hist{n}(k) = E;
    end
end

%% Visualize simulation results
for n = 1:num_bodies
figure;
plot(t,E_hist{n},'k-','LineWidth',2)
ylabel('energy [J]')
xlabel('time $t$ [s]')
title(['Energy plot for body ',num2str(n),': ',...
      params.bodies{n}.body_name,' (',params.bodies{n}.primitive,')'])
axis tight

figure;
subplot(2,2,1)
plot(t,squeeze(Twb_hist{n}(1,1,:)),'r-',...
     t,squeeze(Twb_hist{n}(1,2,:)),'g-',...
     t,squeeze(Twb_hist{n}(1,3,:)),'b-',...
     t,squeeze(Twb_hist{n}(2,1,:)),'r--',...
     t,squeeze(Twb_hist{n}(2,2,:)),'g--',...
     t,squeeze(Twb_hist{n}(2,3,:)),'b--',...
     t,squeeze(Twb_hist{n}(3,1,:)),'r:',...
     t,squeeze(Twb_hist{n}(3,2,:)),'g:',...
     t,squeeze(Twb_hist{n}(3,3,:)),'b:',...
     'LineWidth',2);
axis([min(t) max(t) -1 1])
legend('$R_{11}$','$R_{12}$','$R_{13}$',...
       '$R_{21}$','$R_{22}$','$R_{23}$',...
       '$R_{31}$','$R_{32}$','$R_{33}$',...
       'Location','Best')
ylabel('orientation $R_\mathrm{wb}$');

subplot(2,2,3)
plot(t,Vb_hist{n}(1,:),'r-',...
     t,Vb_hist{n}(2,:),'g-',...
     t,Vb_hist{n}(3,:),'b-',...
    'LineWidth',2);
legend('$\omega_x$','$\omega_y$','$\omega_z$','Location','Best')
ylabel('angular velocity [rad/s] in $\left\{\mathrm{b}\right\}$');
xlabel('time $t$ [s]')

subplot(2,2,2)
plot(t,p_hist{n}(1,:),'r-',...
     t,p_hist{n}(2,:),'g-',...
     t,p_hist{n}(3,:),'b-',...
     'LineWidth',2);
legend('$p_x$','$p_y$','$p_z$','Location','Best')
ylabel('COM position in $\left\{\mathrm{w}\right\}$ [m]');

subplot(2,2,4)
plot(t,v_hist{n}(1,:),'r-',...
     t,v_hist{n}(2,:),'g-',...
     t,v_hist{n}(3,:),'b-',...
    'LineWidth',2);
legend('$v_x$','$v_y$','$v_z$','Location','Best')
ylabel('COM velocity [m/s] in $\left\{\mathrm{w}\right\}$');
xlabel('time $t$ [s]')

sgtitle(['State plots for body ',num2str(n),': ',...
         params.bodies{n}.body_name,' (',params.bodies{n}.primitive,')'])
end

end