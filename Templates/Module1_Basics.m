clear all
close all
clc

%% Module agenda
% In this module we will learn basic MPC formulations in MPT3:
%   1.) How to define model, basic MPC setup, and solve online MPC
%   2.) Compute explicit MPC and plot it results
%   3.) Plant-model mismatch
%   4.) Stability (computation of a Lyapunov function)

%% Task 1a: Basic MPC formulation

% Defining model:
% x(k+1) = A*x(k) + B*u(k)
% y(k)   = C*x(k) + D*u(k)
% --------- Start Modifying Code Here -----------
% A = [...];
% B = [...];
% C = [...];
% D = [...];
% --------- End Modifying Code Here -----------
model = LTISystem('A', A, 'B', B, 'C', C, 'D', D, 'Ts', .1); 

% Constraints
% --------- Start Modifying Code Here -----------
% model.x.max = [...];                     % constraints: x <= xmax 
% model.x.min = [...];                     % constraints: x >= xmin 
% model.u.max = [...];                     % constraints: u <= umax 
% model.u.min = [...];                     % constraints: u >= umin 
% --------- End Modifying Code Here -----------

% MPC setup
% --------- Start Modifying Code Here -----------
% Q = [...];                                % quadratic penalty x'*Q*x
% R = [...];                                % quadratic penalty u'*R*u
% N = ...;                                  % prediction horizon
% --------- End Modifying Code Here -----------
model.x.penalty = QuadFunction(Q);          % quadratic penalty x'*Q*x
model.u.penalty = QuadFunction(R);          % quadratic penalty u'*R*u

%% Task 1b: (online) MPC Formulation + open-loop solution

% Construction of online MPC
mpc = MPCController(model, N);

% Evaluate MPC policy 
% --------- Start Modifying Code Here -----------
% x0 = [...];                               % initial condition
% --------- End Modifying Code Here -----------

% Evaluate the MPC for the given initial condition x0
[u, feasible, openloop] = mpc.evaluate(x0)

% Plot results
figure
subplot(3,1,1)
plot(openloop.X', LineWidth=2)
xlabel('Ts'); ylabel('x1, x2')
subplot(3,1,2)
plot(openloop.Y, LineWidth=2)
xlabel('Ts'); ylabel('y')
subplot(3,1,3)
stairs(openloop.U, LineWidth=2)
xlabel('Ts'); ylabel('u')

%% Task 1c: Closed-loop simulation
% Define a closed-loop system where the controller is the constructed
% online MPC 'mpc' (first argument) and the controlled system is the 
% prediction model 'model' (second argument).

% --------- Start Modifying Code Here -----------
% loop = ClosedLoop(..., ...);            % created closed-loop system
% --------- End Modifying Code Here -----------

% Simulate the closed-loop for 20 steps, starting from state [2; 0]
Nsim = 20;
data = loop.simulate(x0, Nsim)
figure
subplot(2, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x1, x2');
subplot(2, 1, 2)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');

%% Task 2a: Construct (explicit) MPC policy
% Task: compute the empc policy and verify if the explicit solution yields 
% the same solution for the given initial condition x0.

% --------- Start Modifying Code Here -----------
% empc = EMPCController(..., ...);
% [u_empc, feasible_empc, openloop_empc] = empc.evaluate(...)
% --------- End Modifying Code Here -----------

% Plot results
figure
subplot(3,1,1)
hold on
plot(openloop.X', '-b', LineWidth=2)
plot(openloop_empc.X','--r', LineWidth=2)
xlabel('Ts'); ylabel('x1, x2')
subplot(3,1,2)
hold on
plot(openloop.Y, '-b', LineWidth=2)
plot(openloop_empc.Y, '--r', LineWidth=2)
xlabel('Ts'); ylabel('y')
subplot(3,1,3)
hold on
stairs(openloop.U, '-b', LineWidth=2)
stairs(openloop_empc.U, '--r', LineWidth=2)
xlabel('Ts'); ylabel('u')

%% Task 2b: Compare evaluation time of online vs explicit MPC
% Hint: Use 'timeit(@() <command_to_execute>);' to measure evaluation time.
% To evaluate online MPC policy 'mpc' and explicit MPC policy 'empc' use 
% method 'evaluate(x0)' for an arbitrary x0.

% --------- Start Modifying Code Here -----------
% time_empc = timeit(@() ...); % Compute explicit solution
% time_mpc = timeit(@() ...);   % Compute numerical solution
% --------- End Modifying Code Here -----------


[time_empc; time_mpc]
if time_empc < time_mpc
    fprintf(['The explicit MPC is evaluated %f times faster than', ...
        'the online MPC policy!\n'],time_mpc/time_empc)
end

% Q: Why evaluation time of explicit mpc is (generally) faster?

%% Task 3: Plotting explicit solution
% Plot the critical regions
figure; empc.partition.plot(); title('Critical regions');
% Plot the feasible set
figure; empc.partition.Domain.plot(); title('Feasible set');
% Plot the feedback law
figure; 
empc.feedback.fplot(); 
xlabel('x1'); ylabel('x2'); zlabel('u0opt'); title('Feedback law');
% Plot the cost function
figure; 
empc.cost.fplot(); 
xlabel('x1'); ylabel('x2'); zlabel('J'); title('Cost function');

% Q: Can you explain why some space of the X \in [-5;5] is not covered in
% the feasible domain? 

%% Task 4a: Closed-loop simulation
% Perform the same closed-loop simulation with the explicit controller and 
% verify if it returns the same profiles as the online MPC policy. 

% Define a closed-loop system where the controller is the constructed
% explicit/online MPC and controlled system is the prediction model.
% Simulate the closed loop for 20 steps, starting from x0 = [2;0]

% --------- Start Modifying Code Here -----------
% loop_mpc = ClosedLoop(..., ...);
% loop_empc = ClosedLoop(..., ...);
% Nsim = ...;
% x0 = [...];
% data_mpc = loop_mpc.simulate(x0, Nsim)
% data_empc = loop_empc.simulate(x0, Nsim)
% --------- End Modifying Code Here -----------

% Plot results
figure
subplot(3,1,1)
hold on
plot(data_mpc.X', '-b', LineWidth=2)
plot(data_empc.X','--r', LineWidth=2)
xlabel('Ts'); ylabel('x1, x2')
subplot(3,1,2)
hold on
plot(data_mpc.Y, '-b', LineWidth=2)
plot(data_empc.Y, '--r', LineWidth=2)
xlabel('Ts'); ylabel('y')
subplot(3,1,3)
hold on
stairs(data_mpc.U, '-b', LineWidth=2)
stairs(data_empc.U, '--r', LineWidth=2)
xlabel('Ts'); ylabel('u')


%% Task 4b: 
% Q1: Is the closed-loop stable? Repeat the task for a different initial 
%     condition which is from the feasible domain (e.g. [-1 0]')
% --------- Start Modifying Code Here -----------
% x0 = [...];
% data_mpc = loop_mpc.simulate(x0, Nsim)
% data_empc = loop_empc.simulate(x0, Nsim)
% --------- End Modifying Code Here -----------
% Plot results
figure
subplot(3,1,1)
hold on
plot(data_mpc.X', '-b', LineWidth=2)
plot(data_empc.X','--r', LineWidth=2)
xlabel('Ts'); ylabel('x1, x2')
subplot(3,1,2)
hold on
plot(data_mpc.Y, '-b', LineWidth=2)
plot(data_empc.Y, '--r', LineWidth=2)
xlabel('Ts'); ylabel('y')
subplot(3,1,3)
hold on
stairs(data_mpc.U, '-b', LineWidth=2)
stairs(data_empc.U, '--r', LineWidth=2)
xlabel('Ts'); ylabel('u')


%% Task 4c: Stability (recursive feasibility)
% Q2: Are you able to find an initial condition for which the loop would 
%     not be stable? Hint: use the clicksim method. 

% To analyse if the constructed MPC provide recursive feasibility, we can 
% try to find an initial condition x0, from the feasible domain, from which 
% the MPC will not converge to the origin.
empc.clicksim()


%% Task 5a: Calculate a Lyapunov function to certify closed-loop stability
% Verify closed-loop stability by calculating a Lyapunov function.
loop = ClosedLoop(empc, model);
lyap = loop.toSystem.lyapunov('pwq'); % PWQ = piecewise quadratic

% Note: If construction fails, then we do not have a guarantee that the
% system is stable (but it still can be).
% Notice: Which property is violated and is mentioned in the MPT error?

%% Task 5b: 
% Q1: Can we update penalisation matrices Q, R, or modify the prediction 
% N to enforce closed-loop stability? 

% Try to change the MPC setup to enforce recurisve feasibility. Try to
% manipulating with penalisation matrices x'*Q*x and u'*R*u as well as with
% the prediction horizon N.
% Hint: Modify Q/R/N separately. After each modification construct a new
% explicit MPC and use 'clicksim' function to analyse stability.

% --------- Start Modifying Code Here -----------
% model.x.penalty = QuadFunction(...);
% model.u.penalty = QuadFunction(...);
% N = ...;
% --------- End Modifying Code Here -----------
empc = EMPCController(model, N);
empc.clicksim()


% If, based on the clocksim analysis, the system will look stable, try to
% provide a certificate via constructing a Lyapunov funciton. Hint: just
% uncomment the code below.
% --------- Start Modifying Code Here -----------
% loop = ClosedLoop(empc, model);
% lyap = loop.toSystem.lyapunov('pwq'); % PWQ = piecewise quadratic
% figure; lyap.fplot()                  % Show the Lyapunov function
% --------- End Modifying Code Here -----------


%% Task 6: Plant-model mismatch
% Assume that our actuator is demaged and its behaviour has changed
% System matrix is now B = [1; 0.45] (instead of B = [1; 0.5]). 
% Perform closed-loop simulation and visulaize closed-loop performance 
% e.g. from x0 = [-5; 3]. Does the MPC still stalize the controlled system?

% The original (stable) MPC (with a long prediciton horizon)
x0 = [-5; 3]; Nsim = 20;
data = loop.simulate(x0, Nsim);
figure
subplot(2, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x1, x2');
subplot(2, 1, 2)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');

% Closed-loop with the changed matrix B
% --------- Start Modifying Code Here -----------
% Bnew = [...];
% model_changed = LTISystem('A', A, 'B', Bnew, 'C', C, 'D', D, 'Ts', 0.1); 
% --------- End Modifying Code Here -----------
loop2 = ClosedLoop(empc, model_changed);
data = loop2.simulate(x0, Nsim);
subplot(2, 1, 1)
hold on
plot(0:Nsim, data.X,'--', LineWidth=2);
subplot(2, 1, 2)
hold on
stairs(0:Nsim-1, data.U,'--', LineWidth=2);

% To provide a certificate (for the entire MPC domain) we need to calculate
% a Lyapunov function:
lyap = loop2.toSystem.lyapunov('pwq');
