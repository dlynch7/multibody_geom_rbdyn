%% cg4.m
%
% Description:
%   This function implements the 4th-order, 5-stage Runge-Kutta geometric
%   integrator proposed by Crouch and Grossman (J. Nonlinear Sci., Vol. 3,
%   1993).
%
%   This is a geometric integrator, which means it preserves the underlying 
%   geometry of the system being simulated via numerical integration. In
%   the case of a rigid body, its configuration is described in a
%   coordinate-free manner by the 4x4 homogeneous transformation matrix T,
%   which is an element of the special Euclidean group SE(3).
%
%   Rigid body dynamics are a generalization of Newton's 2nd law and
%   Euler's equations for a rotating rigid body and are, in some sense,
%   modeled by a 2nd-order ordinary differential equation (ODE)
%
%   When using an n-dimensional vector q to specify coordinates, one can
%   rewrite a 2nd-order ODE as a system of 1st-order ODEs by defining the
%   state vector x = [q;dqdt], where dqdt is the derivative of the
%   configuration q. In this case, dqdt would be a vector of angular and
%   translational velocities. The resulting system of first-order ODEs is
%           dxdt = f(x),
%   which expands to
%          dq/dt = [dq/dt; (i.e., a trivial ODE)
%      d^2q/dt^2 = g(q,dq/dt)],
%   where d^2q/dt^2 = g(q,dqdt) is the original 2nd-order ODE.
%   
% Inputs:
%   ode_fun: handle to 2nd-order ODE function that computes dVbdt (time derivative
%        of body twist), given t, Twb, Vb, Fb, and params (see below for
%        description of each input argument)
%   dt:  timestep size (scalar)
%   t:   time (scalar)
%   Twb_init: initial configuration of the rigid body's CoM frame {b}
%   relative to the world frame {w}, represented as a 4x4 homogeneous 
%        transformation matrix, i.e., an element of SE(3).
%   Vb_init: initial body twist, represented by a 6x1 vector whose elements
%        are the components of an element of se(3)
%   params: a struct with many fields, generated by calling init_params().
%
% Outputs:
%   t_new: the new time, simply t + dt
%   Twb_new: the new body configuration
%   Vb: the new body twist

function [t_new,Twb_new,Vb_new] = cg4(ode_fun,dt,t_init,Twb_init,Vb_init,params)
%% 4th order, 5-stage Runge-Kutta coefficients
a = zeros(5,5);
b = zeros(5,1);
c = zeros(5,1);

a(2,1) = 1458/1783;

a(3,1) = 1039/3247;
a(3,2) = 97/1470;

a(4,1) = 997/1082;
a(4,2) = 1167/2335;
a(4,3) = -475/433;

a(5,1) = 173/487;
a(5,2) = 751/3141;
a(5,3) = 547/393;
a(5,4) = -680/613;

b(1) = 407/2969;
b(2) = -135/7349;
b(3) = 543/734;
b(4) = -267/1400;
b(5) = 696/2095;

c(1) = 0;
c(2) = 1458/1783;
c(3) = 743/1925;
c(4) = 368/1135;
c(5) = 406/463;

%% Part 1 of 2: compute intermediate values (stages 1 through 5)
s = 5; % GC4 is a 5-stage algorithm

% preallocate arrays:
num_bodies = numel(params.bodies);
Twb_tmp = NaN(4*num_bodies,4,s);
Vb_tmp = NaN(6*num_bodies,s);
K_tmp = NaN(6*num_bodies,s);

% iterate over stage index "s":
for i = 1:s
    Twb_tmp(:,:,i) = Twb_init;
     Vb_tmp(:,i) = Vb_init;
    
    for j = 1:i-1
        % update each 4x4 configuration block separately:
        for n = 1:num_bodies
            Twb_row_start = 1 + (n-1)*4;
            Twb_row_end = Twb_row_start + 3;
            
            Vb_row_start = 1 + (n-1)*6;
            Vb_row_end = Vb_row_start + 5;
            
            Twb_tmp(Twb_row_start:Twb_row_end,:,i) ...
                = squeeze(Twb_tmp(Twb_row_start:Twb_row_end,:,i))...
                * MatrixExp6(dt*a(i,j)*VecTose3(Vb_tmp(Vb_row_start:Vb_row_end,j)));
        end
        Vb_tmp(:,i) = Vb_tmp(:,i) + dt*a(i,j)*K_tmp(:,j);
    end
    K_tmp(:,i) = feval(ode_fun,t_init + c(i)*dt,squeeze(Twb_tmp(:,:,i)),Vb_tmp(:,i));
end

%% Part 2 of 2: compute output from intermediate values

Twb_new = Twb_init;
Vb_new = Vb_init;
for i = 1:s
    % update each 4x4 configuration block separately:
    for n = 1:num_bodies
        Twb_row_start = 1 + (n-1)*4;
        Twb_row_end = Twb_row_start + 3;

        Vb_row_start = 1 + (n-1)*6;
        Vb_row_end = Vb_row_start + 5;

        Twb_new(Twb_row_start:Twb_row_end,:) ...
            = Twb_new(Twb_row_start:Twb_row_end,:)...
            * MatrixExp6(dt*b(i)*VecTose3(Vb_tmp(Vb_row_start:Vb_row_end,i)));
    end
     Vb_new = Vb_new + dt*b(i)*K_tmp(:,i);
end
t_new = t_init + dt;

end