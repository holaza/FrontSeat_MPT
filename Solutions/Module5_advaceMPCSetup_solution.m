clear all
close all
clc

%% Model Setup
% Model
% x(k+1) = A*x(k) + B*u(k)
% y(k)   = C*x(k) + D*u(k)
A = [0.7115 -0.4345; 0.4345 0.8853];
B = [0.2173; 0.0573];
C = [0 1];
D = 0;
% MPC setup parameters
R = 1;
Q = 10*eye(2);
umin = -5;
umax = 5;
xmin = [-2.8; 0];
xmax = [10; 10];
N = 5; % prediction horizon


%% Task 1: Soft constraints
% Construct online MPC policy for the given optimisation problem. Perform 
% closed-loop simulation with yref = 2 for the first 50 steps and yref = 0
% for another 50 steps starting from x_0 = [4 2]'. Assume that at time 
% step 80 a sensor provide a faulty value of the second state with 
% deviation of -1 from the correct value, i.e., at time step 80 perform 
% x_0 = x_0 + [0 -1]'. Plot resulting closed-loop trajectories.
%       Q1: Will MPC handle such a sensor error? Explain why it not.
% 		Q2: Will soft constraints xmin - s <= x_k <= xmax + s, where slacks
%           s >= 0 are heavily penalized, solve this problem? 
%           Hint: Use "model.x.with('softMax')" and 
%                 "model.x.with('softMin')" to add soft constraints on 
%                 states.



% Getting started with Task 1:
% Here we are about to analyse what will happen if our MPC obtaines an
% infeasible initial condition. This can occure e.g. when a sensor will
% incorecctly measure a signal.
%
% A full problem is given below. Try to analyse what went wrong. Update the 
% MPC formulation such that it will be robust against such sensor faulty 
% values.
% Hint: We can use soft constrains "model.signal.with('softMin')" and 
%       "model.signal.with('softMin')" to a correct signal (u, y, or x)

model = LTISystem('A', A, 'B', B, 'C', C, 'D', D); 
model.u.min = umin;                  % constrain u >= umin
model.u.max = umax;                  % constrain u <= umax
model.x.min = xmin;                  % constrain x >= xmin
model.x.max = xmax;                  % constrain x <= xmax
model.u.penalty = QuadFunction(R);   % quadratic penalty u'*R*u
model.y.penalty = QuadFunction(100); % quadratic penalty y'*Qy*y
% Output reference tracking
model.y.with('reference');  % overwrites y'*Q*y -> (y - yref)'*Q*(y - yref) 
model.y.reference = 'free';
% Input reference tracking
model.u.with('reference'); % overwrites u'*R*u -> (u - uref)'*R*(u - uref) 
model.u.reference = 'free'; 

% --------- Start Modifying Code Here -----------
% !!! Before updating this part try firstly to run this section and analyse
% the issue. (expect an error)!!!
% Define soft constraints on states 
model.x.with('softMax');       % upper soft constraints on states
model.x.with('softMin');       % lower soft constraints on states 
% --------- End Modifying Code Here -----------

% Reconstruct the MPC
mpc = MPCController(model, N);

% Create a vector of references for the entire simulation Nsim = 100
Nsim = 100;
yref1 = 2;
yref2 = 0;
Yref = [ones(1,Nsim/2)*yref1, ones(1,Nsim/2)*yref2];
Uref = B\(eye(2)-A)*(C\Yref); 

% Perform simulation
x0 = [4; 2];
Y = []; U = []; X = x0;
model.initialize(x0);    % initialize the model from x0
for i = 1:Nsim
    u = mpc.evaluate(x0, 'y.reference', Yref(:,i), 'u.reference', Uref(:,i));
    if isnan(u)
        subplot(3, 1, 1)
        plot(0:i-1, X, LineWidth=2); xlabel('Ts'); ylabel('x');
        hold on
        plot([0 i-1], [model.x.min(2), model.x.min(2)]', '--k', LineWidth=2);
        legend('x1','x2','x1_{mincon}')
        subplot(3, 1, 2)
        plot(0:i-2, Y, LineWidth=2); xlabel('Ts'); ylabel('y');
        hold on
        plot(0:Nsim-1, Yref, 'r--', LineWidth=2);
        plot([0 i-1], [model.x.min(2), model.x.min(2)]', '--k', LineWidth=2);
        legend('y','yref','ycon')
        subplot(3, 1, 3)
        stairs(0:i-2, U, LineWidth=2); xlabel('Ts'); ylabel('u');
        error('Ups, the MPC problem is infeasible!');
    end
    % If an error occures => change it to: [x0, y] = model.update(x0,u);

    [x0, y] = model.update(x0,u); % Different versions of MPT3 might need 
                                  % to use "[x0, y] = model.update(u);"
    if i == Nsim - 20, x0 = x0 + [0; -1]; end  % sensor failure
    X = [X, x0];
    U = [U; u];
    Y = [Y; y];
end

% plot results
subplot(3, 1, 1)
plot(0:Nsim, X, LineWidth=2); xlabel('Ts'); ylabel('x');
subplot(3, 1, 2)
plot(0:Nsim-1, Y, LineWidth=2); xlabel('Ts'); ylabel('y');
hold on
plot(0:Nsim-1, Yref, 'r--', LineWidth=2);
plot([0 Nsim], [model.x.min(2), model.x.min(2)]', '--k', LineWidth=2);
legend('y','yref','ycon')
subplot(3, 1, 3)
stairs(0:Nsim-1, U, LineWidth=2); xlabel('Ts'); ylabel('u');

% Note: We have used the online MPC (not explicit). This reason is that
% complexity of the explicit increases exponentionaly and it takes some
% time to construct it. Hence, to save time, we have used the online MPC.

%% Task 2: Delta-u constraints
% Construct online MPC policy that will restrict change of the control 
% action in two subsequent steps to +-1, i.e., we want to add 
% -1 <= u_k - u_{k-1} <= 1. Perform closed-loop simulation and verify if 
% this delta-u constraint is satisfied.


% Notice how agresive was the control action at step 0 and 50. This can
% lead to actuator damage. Hence, we aim to restrict the control action to
% min/max change (i.e. control action will be able to change only by a 
% specific value between two subsequent samples).
%
% Try to execude this part of the code. Try to manipulate with bounds 
% 'model.u.deltaMin' and 'model.u.deltaMax' as well as with 'u0'.  
% Analyse results.

model = LTISystem('A', A, 'B', B, 'C', C, 'D', D); 
model.u.min = umin;                  % constrain u >= umin
model.u.max = umax;                  % constrain u <= umax
model.x.min = xmin;                  % constrain x >= xmin
model.x.max = xmax;                  % constrain x <= xmax
model.u.penalty = QuadFunction(R);   % quadratic penalty u'*R*u
model.y.penalty = QuadFunction(100); % quadratic penalty y'*Qy*y
% Output reference tracking
model.y.with('reference'); % overwrites y'*Q*y -> (y - yref)'*Q*(y - yref) 
model.y.reference = 'free';
% Input reference tracking
model.u.with('reference');  % overwrites u'*Q*u -> (u - uref)'*Q*(u - uref) 
model.u.reference = 'free'; 

% --------- Start Modifying Code Here -----------
% Min/max constraints on delta-u
model.u.with('deltaMin');                                                  
model.u.with('deltaMax');                                                  
model.u.deltaMin = -0.5;     
model.u.deltaMax = 0.5;
% --------- End Modifying Code Here -----------
                                                      
% Reconstruct the MPC
mpc = MPCController(model, N);

% Create a vector of references for the entire simulation Nsim = 100
Nsim = 100;
yref1 = 2;
yref2 = 1;
Yref = [ones(1,Nsim/2)*yref1, ones(1,Nsim/2)*yref2];
Uref = B\(eye(2)-A)*(C\Yref); 

% perform simulation
x0 = [4; 2]; u0 = 3;
loop = ClosedLoop(mpc, model);
data = loop.simulate(x0, Nsim, 'y.reference', Yref, 'u.reference', ...
    Uref, 'u.previous', u0); 
figure
subplot(3, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x');
subplot(3, 1, 2)
plot(0:Nsim-1, data.Y, LineWidth=2); xlabel('Ts'); ylabel('y');
hold on
plot(0:Nsim-1, Yref, 'r--', LineWidth=2);
subplot(3, 1, 3)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');

% Notice1: that control action does not exceed deltaMin/deltaMax value in
% subsequent steps. However, this is at the cost of performance.
% Notice2: We need to provide to the MPC another parameter 'u.previous'.

%% Task 3: Delta-u penalty
% Construct online MPC policy that will penalize the change of the control 
% action in two subsequent steps by a penalty matrix H, i.e., 
% (u_k - u_{k-1})' H (u_k - u_{k-1}). Verify how the matrix H affects the 
% closed-loop performance.  
% Q1:  Can we remove the 'u.reference' formulation and will the MPC track 
%      the reference without an offset? In another words, can we replace 
%      (u_k - uref)' R (u_k - uref) with 
%      (u_k - u_{k-1})' H (u_k - u_{k-1})? 
%      Hint: Comment the "model.u.with('reference')" and 
%            "model.u.reference = 'free';". Moreover, in the function 
%            "simulate()" function remove "'u.reference', Uref,". 
%            Then rerun the scenario.


%% Task 3a: Adding delta-u penalty
% A similar results, as with the delta-u constraints, can be obtained by 
% changing the objective function of the MPC. Specifically, we can add 
% another term (u - u_k-1)'*H*(u - u_k-1), where u_k-1 is the control 
% action from the previous step.
%
% Try to execude this part of the code. Try to manipulate with the 
% deltaPenalty. Analyse results.

model = LTISystem('A', A, 'B', B, 'C', C, 'D', D); 
model.u.min = umin;                  % constrain u >= umin
model.u.max = umax;                  % constrain u <= umax
model.x.min = xmin;                  % constrain x >= xmin
model.x.max = xmax;                  % constrain x <= xmax
model.u.penalty = QuadFunction(R);   % quadratic penalty u'*R*u
model.y.penalty = QuadFunction(100); % quadratic penalty y'*Qy*y
% Output reference tracking
model.y.with('reference'); % overwrites y'*Q*y -> (y - yref)'*Q*(y - yref) 
model.y.reference = 'free';
% Input reference tracking
model.u.with('reference');  % overwrites u'*Q*u -> (u - uref)'*Q*(u - uref) 
model.u.reference = 'free'; 

% --------- Start Modifying Code Here -----------
% Delta-u penalty             
H = 10;
model.u.with('deltaPenalty'); 
model.u.deltaPenalty = QuadFunction( H );    % (u - u_k-1)'*H*(u - u_k-1)
% --------- End Modifying Code Here -----------

% Reconstruct the MPC
mpc = MPCController(model, N);

% Create a vector of references for the entire simulation Nsim = 100
Nsim = 100;
yref1 = 2;
yref2 = 1;
Yref = [ones(1,Nsim/2)*yref1, ones(1,Nsim/2)*yref2];
Uref = B\(eye(2)-A)*(C\Yref); 

% perform simulation
x0 = [4; 2]; u0 = 3;
loop = ClosedLoop(mpc, model);
data = loop.simulate(x0, Nsim, 'y.reference', Yref, 'u.reference', ...
    Uref, 'u.previous', u0);   %<---- changed here 
figure
subplot(3, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x');
subplot(3, 1, 2)
plot(0:Nsim-1, data.Y, LineWidth=2); xlabel('Ts'); ylabel('y');
hold on
plot(0:Nsim-1, Yref, 'r--', LineWidth=2);
subplot(3, 1, 3)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');

%% Task 3b: Removing uref formulation (when having delta-u penalty)
% Q: Can we remove the 'u.reference' formulation and will the MPC track 
%    the reference without an offset? In another words, can we replace
%    (u - uref)'*R*(u - uref) with (u - u_prev)'*R*(u - u_prev).
% Hint: Comment the 
%           model.u.with('reference');
%           model.u.reference = 'free';
%       and in the simulate() function remove "'u.reference', Uref,".
%       Reexecute and analyse results.

model = LTISystem('A', A, 'B', B, 'C', C, 'D', D); 
model.u.min = umin;                  % constrain u >= umin
model.u.max = umax;                  % constrain u <= umax
model.x.min = xmin;                  % constrain x >= xmin
model.x.max = xmax;                  % constrain x <= xmax
model.u.penalty = QuadFunction(R);   % quadratic penalty u'*R*u
model.y.penalty = QuadFunction(100); % quadratic penalty y'*Qy*y
% Output reference tracking
model.y.with('reference'); % overwrites y'*Q*y -> (y - yref)'*Q*(y - yref) 
model.y.reference = 'free';

% --------- Start Modifying Code Here -----------
% Input reference tracking
% Delta-u penalty             
H = 10;
model.u.with('deltaPenalty'); 
model.u.deltaPenalty = QuadFunction( H );    % (u - u_k-1)'*H*(u - u_k-1)
% --------- End Modifying Code Here -----------
                                
% Reconstruct the MPC
mpc = MPCController(model, N);

% Create a vector of references for the entire simulation Nsim = 100
Nsim = 100;
yref1 = 2;
yref2 = 1;
Yref = [ones(1,Nsim/2)*yref1, ones(1,Nsim/2)*yref2];
Uref = B\(eye(2)-A)*(C\Yref); 

% perform simulation
x0 = [4; 2]; u0 = 3;
loop = ClosedLoop(mpc, model);
data = loop.simulate(x0, Nsim, 'y.reference', Yref, 'u.previous', u0);
figure
subplot(3, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x');
subplot(3, 1, 2)
plot(0:Nsim-1, data.Y, LineWidth=2); xlabel('Ts'); ylabel('y');
hold on
plot(0:Nsim-1, Yref, 'r--', LineWidth=2);
subplot(3, 1, 3)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');

%% Task 4: Move-blocking
% Construct online MPC policy that uses move-blocking technique with 
% blocked control action from N = 3 to N = 5. In another words, we want to 
% force the last 3 optimized control actions to be identical u_2=u_3=u_4. 
% Perform closed-loop simulation and analyse how this technique affects 
% control performance. 
% Q: Are we able to construct explicit controller of the original problem 
%    (without the move blocking technique)? Try to change complexity of the 
%    MPC problem via modifying the length of blocking interval (as well as 
%    the N) and try to construct the explicit solution.


% It is a standard practice that to reduce complexity of MPC we can use so
% called move-blocking method. Here, we specify whihch contorl ations will
% be the same as the previous one. For example move blocking from Nc to N, 
% e.g. 3 to 5, means that u_1, u_2, and u_3 can be unique, but 
% u_3 = u_4 = u_5 are the same.
% 
% Try to execute the code. Manipulate with N and Nc and analyse how the
% performance changes. Try to set Nc and N, such that we will be able
% to calculate the eplicit solution 'empc = mpc.toExplicit()' in a
% reasonable time (i.e. within 1 minute).

model = LTISystem('A', A, 'B', B, 'C', C, 'D', D); 
model.u.min = umin;
model.u.max = umax;
model.x.min = xmin;
model.x.max = xmax;
model.y.penalty = QuadFunction(100);
% Output reference tracking
model.y.with('reference'); 
model.y.reference = 'free';
% Delta-u penalty                                              
model.u.with('deltaPenalty');                                              
model.u.deltaPenalty = QuadFunction( 1 );                                   

% --------- Start Modifying Code Here -----------
% Setup:
N = 10;     % prediction horizon
Nc = 5;     % control horizon
% Add Move-blocking
model.u.with( 'block' ); 
model.u.block.from = Nc;                                                   
model.u.block.to = N;
% --------- End Modifying Code Here -----------

% Reconstruct the MPC
mpc = MPCController(model, N);


% Verify if the move blocking works.
% Hint: Evaluate the mpc policy for a feasible initial condition and plot
% open-loop control sequence. (A good choice of parameters is e.g.: 
% N = 10, Nc = 5, x0 = [0;0], u0 = 3, yref = 1)
% --------- Start Modifying Code Here -----------
x0 = [0;0];
u0 = 3;
yref = 1;
[u, feas, info] = mpc.evaluate(x0, 'y.reference', yref, 'u.previous', u0)
figure
title('Open-loop sequence of control actions')
stairs(info.U, LineWidth=2)
title('Open-loop control actions')
% Answer: notice that first 5 control actions are allowed to attain any
% (optimal) value, but the last 5 contorl actions are the same (as Nc = 5);
% --------- End Modifying Code Here -----------


% Create a vector of references for the entire simulation Nsim = 100
Nsim = 100;
yref1 = 2;
yref2 = 1;
Yref = [ones(1,Nsim/2)*yref1, ones(1,Nsim/2)*yref2];
Uref = B\(eye(2)-A)*(C\Yref); 
% perform simulation
x0 = [4; 2]; u0 = 3;
loop = ClosedLoop(mpc, model);
data = loop.simulate(x0, Nsim, 'y.reference', Yref, 'u.previous', u0);   
figure
subplot(3, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x');
subplot(3, 1, 2)
plot(0:Nsim-1, data.Y, LineWidth=2); xlabel('Ts'); ylabel('y');
hold on
plot(0:Nsim-1, Yref, 'r--', LineWidth=2);
subplot(3, 1, 3)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');
