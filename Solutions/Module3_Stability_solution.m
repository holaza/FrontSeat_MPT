clear all
close all
clc

%% Module 3: Stability
% In this module we will use the same (original) MPC setup, as in the first
% module, of what we know that it is not stable. In this module we will 
% show which ingredients we need to add the the MPC policy to provide 
% guarantees of stability.

%% Defining the MPC problem
% Model
% x(k+1) = A*x(k) + B*u(k)
% y(k)   = C*x(k) + D*u(k)
A = [1 1; 0 1];
B = [1; 0.5];
C = [1 0];
D = 0;
model = LTISystem('A', A, 'B', B, 'C', C, 'D', D, 'Ts', 0.1); 

% Constraints
model.x.max = [5; 5];                       % constraints: x <= xmax 
model.x.min = [-5; -5];                     % constraints: x >= xmin 
model.u.max = 1;                            % constraints: u <= umax 
model.u.min = -1;                           % constraints: u >= umin

% MPC setup
model.x.penalty = QuadFunction(eye(2));     % quadratic penalty x'*Q*x
model.u.penalty = QuadFunction(1);          % quadratic penalty u'*R*u
N = 3;                                      % prediction horizon

% Calculate the explicit MPC
empc = EMPCController(model, N);

% From the previous module we know that such constructed MPC does not
% guaranty stability. To enforce the stability, we can add terminal set and
% penalty to the optimisation problem. We will show that such constructed 
% MPC policy stabilizes the system. 

%% Task 1: Terminal set
% Add terminal set to the MPC setup.  
% Hint: The terminal set can be computed in MPT via model.LQRSet)

% Add terminal set to the MPC
model.x.with('terminalSet')
model.x.terminalSet = model.LQRSet;


%% Task 2: Terminal penalty
% Add terminal penalty to the MPC setup.  
% Hint: The terminal penalty is solution of the Ricatty equation, 
% i.e. unconstrained MPC problem.

% --------- Start Modifying Code Here -----------
model.x.with('terminalPenalty');
P = model.LQRPenalty;   % or we can use [K,P] = dlqr(...)
model.x.terminalPenalty = P;
% --------- End Modifying Code Here -----------

%% Task 3: Trial and error analysis of closed-loop stability
%  Construct a new explicit MPC feedback law and via clicksim function 
% analyse if the controller stabilizes the controlled system (prediction 
% model).

% Consruct explicit MPC
empc_stable = EMPCController(model, N);
% use clicksim function to analyse stability
empc_stable.clicksim()

%% Task 4: Stability certificate
% Provide a certificate of stability (via constructing Lyapunov function).

% --------- Start Modifying Code Here -----------
loop = ClosedLoop(empc_stable,model);
lyap = loop.toSystem.lyapunov('pwq');
% --------- End Modifying Code Here -----------

%% Task 5: Further analysis
% Provide answer to following questions:
% Q1: Can we choose arbitrary prediction horizon and the MPC (with terminal
% set/penalty) will always guarantee stability?
% Q2: Will volume of the MPC feasible domain increase/decrease with the
% prediction horizon N?
% Q3: Can we change volume of the feasibility domain by changing N to 
% infinity?
%
% HINT:
% Compute 3 MPC policies with different prediction horizon N, e.g. {3,5,10} 
% To each of them try to:
%   a) construct a PWQ Lyapunov function (answer to Q1).
%   b) plot their feasible sets and compare their volumes (answer to Q2).
%   c) compute explicit MPC with large prediciton horizon (e.g. N = 100)
%      and visually compare its domain with previous MPCs (answer to Q3).

%% Task 5a: 
% Can we choose arbitrary prediction horizon and the MPC (with terminal
% set/penalty) will always guarantee stability?

% --------- Start Modifying Code Here -----------
empc_stable1 = EMPCController(model, 3);
empc_stable2 = EMPCController(model, 5);
empc_stable3 = EMPCController(model, 10);
% --------- End Modifying Code Here -----------
loop = ClosedLoop(empc_stable1, model); loop.toSystem.lyapunov('pwq');
loop = ClosedLoop(empc_stable2, model); loop.toSystem.lyapunov('pwq');
loop = ClosedLoop(empc_stable3, model); loop.toSystem.lyapunov('pwq');

%% Task 5b:
% Will volume of the MPC feasible domain increase/decrease with the
% prediction horizon N?

figure
hold on
empc_stable3.partition.Domain.plot('color','g');
empc_stable2.partition.Domain.plot('color','b');
empc_stable1.partition.Domain.plot('color','r');
legend('MPC with N3','MPC with N2','MPC with N1');

%% Task 5c:
% Can we change volume of the feasibility domain by changing N to infinity?

% --------- Start Modifying Code Here -----------
% Let us construct MPC with a very long prediction horizon
empc_stable_longN = EMPCController(model, 100);
% --------- End Modifying Code Here -----------

% Graphical representation
figure
hold on
empc_stable_longN.partition.Domain.plot('color','k');
empc_stable3.partition.Domain.plot('color','g');
empc_stable2.partition.Domain.plot('color','b');
empc_stable1.partition.Domain.plot('color','r');
legend('MPC with Nlong','MPC with N3','MPC with N2','MPC with N1')

% Calculating volume of each domain.
VlongN = empc_stable_longN.partition.Domain.volume;
V3 = empc_stable3.partition.Domain.volume;
V2 = empc_stable2.partition.Domain.volume;
V1 = empc_stable1.partition.Domain.volume;
fprintf('Volumes of domains are: %f \n',[VlongN; V3; V2; V1])

% Answer:
% In the figure we can observe that the explicit policy with N = 100 has
% the same domain as the explicit mpc with N = 10.
% Hence, untill a certain point constraints will not allow the domain to 
% expand any more. This means that increasing N makes sense only to a 
% certain point when the max volume is reached. (Also no performance 
% improvements will be achieved)
