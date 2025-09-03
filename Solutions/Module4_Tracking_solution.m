clear all
close all
clc

%% Defining the MPC problem

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

% Prediction model
% x(k+1) = A*x(k) + B*u(k)
% y(k)   = C*x(k) + D*u(k)
model = LTISystem('A', A, 'B', B, 'C', C, 'D', D); 

% MPC setup
model.u.min = umin;                     % constraints: u >= umin
model.u.max = umax;                     % constraints: u <= umax
model.x.min = xmin;                     % constraints: x >= xmin
model.x.max = xmax;                     % constraints: x <= xmax
model.u.penalty = QuadFunction(R);      % quadratic penalty u'*R*u
model.x.penalty = QuadFunction(Q);      % quadratic penalty x'*Q*x

%% Task 1:Regulation problem
% Obtain the explicit representation of the MPC problem and perform 
% closed-loop simulation from x_0 = [4 2]' with 20 steps. Plot resulting 
% trajectories. (Note: this is just a regulation problem.)

% --------- Start Modifying Code Here -----------
% Calculate the explicit MPC
empc = EMPCController(model, N);
% Perfroming regulation from x0 = [4 2]'
x0 = [4;2]; 
Nsim = 20;
loop = ClosedLoop(empc, model);
data = loop.simulate(x0, Nsim);
% --------- End Modifying Code Here -----------

% Plot results
figure
subplot(2, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x1, x2');
subplot(2, 1, 2)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');

%% Task 2: Tracking to a predefined reference 
% Instead of regulating towards the origin, we want the MPC controller to 
% steer the system's states to xref = [0 2]'. To do so, modify the stage 
% cost to (x_k - xref)'*Q*(x_k - xref)' + u_k'*R*u_k. Verify the 
% performance of the controller in a closed-loop simulation starting 
% from x_0 = [4 2]'.

% Here we will modify the MPC problem to track a pre-defined state
% reference xref = [0 2]'.
xref = [0;2];                                   
model.x.with('reference');  % defines penalty: (x-xref)'*Q*(x-xref)
model.x.reference = xref;   % specifying a fixed reference

% Calculate the explicit MPC
empc = EMPCController(model, N);

% perform simulation
loop = ClosedLoop(empc, model);
data = loop.simulate(x0, Nsim);
figure
subplot(2, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x1, x2');
hold on
plot(0:Nsim, data.X.*0 + xref, 'r--', LineWidth=2);
subplot(2, 1, 2)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');

% Q1: Is the reference tracked without a steady-state offset? Explain why.
% The steady state error (SSE) is calculated as:
% --------- Start Modifying Code Here -----------
SSE = xref - data.X(:,end)
% --------- End Modifying Code Here -----------
% Hint: our objective function is 
%                 J_k = (x_k - xref)'*Q*(x_k - xref) + u_k'*R*u_k
% and we are trying to minimize it. Can we reach J_k == 0?

% Answer: Tracking to a non-zero reference requires a non-zero contorl
% action. This meas that if we want to achieve (x_k - xref)'*Q*(x_k - xref)
% to be equal to zero then our states need to be at hte reference
% specifically. However, as we require a non-zero input the get to the
% xref, the term u_k'*R*u_k will not be equal to zero. Finally, the
% optimisation problem is trying to minimize both terms at the same time.
% This means that both terms will be non-zero => x_k ~= xref.


% Q2: Is dimensionality of the feasible set (of the explicit controller) 
%     still sthe same dimension (2D) as with the regulation problem? Why?
empc.partition.Dim   % Dimensionalty 
% Hint: Dimensionality increases only if we need to add a new value to
% solve the optimisation problem. Notice that function 'simulate' did not
% required a new parameter.

% Answer: 
% The dimensionality of the explicit solution is the same. THe reason is
% that we do not need to provide to the optimisation problem an additional
% parameter (the xref) as it is constant.

%% Task 3: Offset-free tracking
% To reject the steady-state offset, calculate the target control input 
% uref such that xref = A*xref + B*uref holds, and change the stage cost 
% to (x_k - xref)'*Q*(x_k - xref) + (u_k - uref)'*R*(u_k - uref). 
% Verify the performance in a closed-loop simulation. Why is the 
% steady-state offset rejected?

% --------- Start Modifying Code Here -----------
% Calculate the steady-state control input corresponding to xref
uref = B\(eye(model.nx) - A)*xref;
% Enable tracking of non-zero references
model.u.with('reference');        % defines penalty: (u-uref)'*R*(u-uref)
model.u.reference = uref;         % fixed input reference
% --------- End Modifying Code Here -----------

% Create a new controller
empc = EMPCController(model, N);
% Simulate the closed loop
loop = ClosedLoop(empc, model);
data = loop.simulate(x0, Nsim);
figure
subplot(2, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x1, x2');
hold on
plot(0:Nsim, data.X.*0 + xref, 'r--', LineWidth=2);
title('Closed-loop simulation')
legend('x1','x2')
subplot(2, 1, 2)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');

%% Task 4: Tracking output reference
% Create explicit MPC controller to track the output reference yref = 2 
% such that x = A x + B uref and yref = C x + D uref hold. To do this we 
% need to change the objective function to (y_k - yref)' Q (y_k - yref) + 
% + (u_k - uref)' R (u_k - uref). Verify the performance in a closed-loop 
% simulation. Is still the steady-state offset rejected?

% Hint: In the 'model' define that the 'y' has reference (not the 'x').
% Moreover, keep in mid that MPC has to be "motivated" to go to the
% reference.

% --------- Start Modifying Code Here -----------
model = LTISystem('A', A, 'B', B, 'C', C, 'D', D); 
model.u.min = umin;
model.u.max = umax;
model.x.min = xmin;
model.x.max = xmax;
model.y.penalty = QuadFunction(100);       % (y-yref)'*Q*(y-yref)         
model.u.penalty = QuadFunction(1);          % quadratic penalty u'*Q*u
model.y.with('reference');        % adding reference tracking to the MPC     
yref = 2;                                                                  
model.y.reference = yref;         % specifying a fixed reference 
% To eliminate the offset we will also use the uref formulation
uref = B\(eye(model.nx) - A)*(C\yref);                                             
model.u.with('reference');          % defines penalty: (u-uref)'*R*(u-uref)
model.u.reference = uref;           % fixed input reference
% --------- End Modifying Code Here -----------

empc = EMPCController(model, N);
loop = ClosedLoop(empc, model);
data = loop.simulate(x0, Nsim);
figure
subplot(2, 1, 1)
plot(0:Nsim-1, data.Y, LineWidth=2); xlabel('Ts'); ylabel('x1, x2');
hold on
stairs([0 Nsim], [yref yref], 'r--', LineWidth=2);
subplot(2, 1, 2)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');

%% Task 5: Tracking with a free reference
% Create explicit MPC policy that tracks an arbitrary output reference 
% yref \in R. Verify the performance in a closed-loop simulation with 
% yref = 2 for the first 50 steps and yref = 1 for another 50 steps.
%
% Q1: Can we define a negative reference? Why not?
% Q2: Is dimensionality of the feasible set (of the explicit controller) 
%     still the same as with the regulation problem? Why? 
%     Hint: Use 'empc.partition.Dim' to determine dimensionality of the 
%           polyhedral partition.


% specifying a free output reference
model.y.with('reference'); 
model.y.reference = 'free';   
% specifying a free input reference 
model.u.with('reference');
model.u.reference = 'free';    
% Calculate the explicit MPC
empc = EMPCController(model, N);

% --------- Start Modifying Code Here -----------
% Create a vector of references for the entire simulation Nsim = 100
Nsim = 100;
yref1 = 2;
yref2 = 1;
Yref = [ones(1,Nsim/2)*yref1, ones(1,Nsim/2)*yref2];   % This vector should have dimensionality of 1x100
Uref =  B\(eye(model.nx) - A)*(C\Yref);              % This vector should have dimensionality of 1x100
% --------- End Modifying Code Here -----------




% perform simulation
x0 = [4; 2];
loop = ClosedLoop(empc, model);
data = loop.simulate(x0, Nsim, 'y.reference', Yref, 'u.reference', Uref);   
figure
subplot(3, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x');
subplot(3, 1, 2)
plot(0:Nsim-1, data.Y, LineWidth=2); xlabel('Ts'); ylabel('y');
hold on
plot(0:Nsim-1, Yref, 'r--', LineWidth=2);
subplot(3, 1, 3)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');


%% Q1: Can we define a negative reference? Why not?
% Hint: Change the second referenc from 'yref2 = 1' to 'yref2 = -1' and
%       rerun the secion. Try to explain results.
% 
% Answer: Yes, we can define a negative value of the y-reference. THe
% problem is that from the model C matric we know that the output
% corresponds to the second state. Next, see that the second state is
% constrained as 0 <= x2 <= 10. This means that also the output y=x2 will
% be constrained by 0 <= y <= 10. Finally, the consequence is that the MPC 
% will try to go towards the yref, but it will hit the constraint y = 0 and
% do not descend further. 
%

Nsim = 100;
yref1 = 2;
yref2 = -1;
Yref = [ones(1,Nsim/2)*yref1, ones(1,Nsim/2)*yref2];   % This vector should have dimensionality of 1x100
Uref =  B\(eye(model.nx) - A)*(C\Yref);  

x0 = [4; 2];
loop = ClosedLoop(empc, model);
data = loop.simulate(x0, Nsim, 'y.reference', Yref, 'u.reference', Uref);   
figure
subplot(3, 1, 1)
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x');
subplot(3, 1, 2)
plot(0:Nsim-1, data.Y, LineWidth=2); xlabel('Ts'); ylabel('y');
hold on
plot(0:Nsim-1, Yref, 'r--', LineWidth=2);
subplot(3, 1, 3)
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');

%% Q2: 
% Is dimensionality of the feasible set (of the explicit controller) 
% still the same as with the regulation problem (2D)? Why not?
empc.partition.Dim  

% Answer: The dimensionalty is 4D as we have these parameters: 
% [x1 x2 yref uref]. In other words, we need to provide t othe optimisation
% problem the vector [x1 x2 yref uref] (see that we have done it in the 
% .simulate() function).



