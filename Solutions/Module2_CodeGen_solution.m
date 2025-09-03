clear all
close all
clc

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

%% Exporting Parametric Solution into a Stand Alone Code
% MPT3 allows to export parametric solutions in three ways:
%    1.) Export to pure Matlab code
%    2.) Export to pure C code
%    3.) Export to pure Python code
% Moreover, it will be shown how to export matrices of MPC optimisation
% problem.

%% Task 1: Construct the explicit solution of the MPC problem
% Calculate the explicit MPC
empc = EMPCController(model, N);

% We want to export the controller that is hidden under 'empc.optimizer'.
% Let us firstly analise what is the optimizer telling us:
empc.optimizer

% We can read that:
%   1. The domain is created as a polyunion in 2D with 9 polyhedra. We can
%   see it via typing: 
%                   figure; empc.optimizer.Domain.plot()
%   2. There are properties of the domain (convex, bounded, ...)
%   3. Above this domain are defined several functions. We are interested
%   in the feedback law named as "primal". We can look at it via:
%           figure; empc.optimizer.getFunction('primal').fplot
%   There is also the cost function "obj" which we can visually analyse via
%            figure; empc.optimizer.getFunction('obj').fplot

%% Task 2a: Export to standalone MATLAB code:
% To export a stand alone MATLAB code, we can use following command:
% empc.optimizer.toMatlab(filename, function, tiebreak)
% where:
%   filename - is the name of the created function (e.g. 'empc_fun.m')
%   function - function to be exported (e.g. 'primal')
%   tiebreak - name of function to use to resolve tiebreaks (e.g. 'obj')
%              (use 'first-region' to break the sequential search once
%              the first region containing a given point is found)

% Exporting explicit MPC to pure MATLAB code
% --------- Start Modifying Code Here -----------
file_name = 'empcfun.m';    % do not forget to add ".m" in the end
function_to_export = 'primal';
empc.optimizer.toMatlab(file_name, function_to_export, 'first-region')
% --------- End Modifying Code Here -----------

% We can analyse that the generated code performs as:
% for k = 1:...             % itterate through all regions
%   if all(A_k*x <= b_k)    % checking if the state x lays in k-th reagion
%      u = F_k*x + g_k;     % control law evaluation
%      break;               % stop the algorithm if active region was found
%   end
% end

% Evaluate the exported function for a particular initial condition. For
% example if the exported file is named 'empc_in_matlab.m' then:
% --------- Start Modifying Code Here -----------
x0 = [2;0];
[Uopt, region] = empcfun(x0)        % modify name of the fuction 
% --------- End Modifying Code Here -----------
% where 'Uopt' is the open-loop control sequence and 'region' is the active
% region, i.e., polyhedron that contains the state x0. 

%% Task 3: Trim the exported control law
% Notice that the exported function now returns the entire open-loop
% sequence of control actions. But we only apply the first control action
% from that sequence.
%
% We can use trimFunction to specify, how many outputs should return a
% specific function. Specify which function should be trimmed (the first 
% argument) and how many outputs should it provide (the second argument),
% i.e., empc.optimizer.trimFunction(<function name>, <number of outputs>)


% --------- Start Modifying Code Here -----------
number_of_outputs = 1;
empc.optimizer.trimFunction(function_to_export, number_of_outputs);

% Now we can re-export the function
file_name2 = 'empcfuntrim';   % use a different funciton name to export
empc.optimizer.toMatlab(file_name2, function_to_export, 'first-region')

% Verify that both functions retun the same RHC 
[Uopt, region] = empcfun(x0)
[Uopt_trim, region_trim] = empcfuntrim(x0)
% --------- End Modifying Code Here -----------


% Q: Can you calculate how much memory (numbers) we will save if the 
% trimmed matrices will be used? (Hint: Either use rigorous calculations or
% simply compare matrices [H, ni, fF, fg] in both m-files.)

%% Task 4: Closed-loop simulation
% Create a closed-loop simulation with the created script, where the
% controoled system is the prediction model itself, i.e. model.A and 
% model.B. Compare the solution with the result from using build in MPT3 
% functions from previous module, i.e., use the original 'empc' constorller
% and functions 'ClosedLoop' + 'simulate'.
%
% Note: Be aware that output from the function returns the entire open-loop
% sequence of control inputs. We need to use only the first control input.

% Simulation setup
x0 = [3; 0]; Nsim = 20;

% Perform closed-loop simulation with the MPT3 
empc.N = 1;    % overwrite the prediction horizon of the trimmed function
loop = ClosedLoop(empc, model);
data = loop.simulate(x0, Nsim);

% Perform closed-loop simulation with the created script
% --------- Start Modifying Code Here -----------
U = zeros(model.nu,Nsim);       % list of all (RHC) control actions
X = zeros(model.nx,Nsim+1);     % list of all states
R = zeros(Nsim,1);              % list of active regions
X(:,1)  = x0;
for k = 1:Nsim
    % Evaluate the MPC (script)
    [Uopt, R(k)] = empcfun(X(:,k));
    if R(k) == 0, error('No region for state %s.', mat2str(X(:,k))); end

    % Extract the RHC law
    U(:,k) = Uopt(1:model.nu);

    % Apply control actionv and calculate new state: x+ = A*x + B*u
    X(:,k+1) = model.A*X(:,k) + model.B*U(:,k);
end
% --------- End Modifying Code Here -----------

% Plot the results:
figure
subplot(2, 1, 1)
hold on
plot(0:Nsim, data.X, LineWidth=2); xlabel('Ts'); ylabel('x1, x2');
plot(0:Nsim, X, '--r', LineWidth=2);
subplot(2, 1, 2)
hold on
stairs(0:Nsim-1, data.U, LineWidth=2); xlabel('Ts'); ylabel('u');
stairs(0:Nsim-1, U, '--r', LineWidth=2); xlabel('Ts'); ylabel('u');
    
%% Task 5: Export to C
%   We need to used function:
%           empc.optimizer.toC('function', 'output','tie_break_fcn')
%   where:
%   'function' - refers to the name of attached function to export 
%                (can be only PWA or PWQ function)
%   'output' - file name to be generated
%   'tie_break_fcn' - refers to the name of attached function that will be 
%                     used for comparison if the function is not uniquely 
%                     defined
%                   - the returned function value is the one with the 
%                     minimum value of the tie-breaking function
%  Generated code contains two C-files. The first file identified with the
%  function name is the sole implementation of the sequential search
%  algorithm with the data of the PWA/PWQ function. The second file is a
%  C-mex interface for fast evaluation inside Matlab for test purposes.


% Exporting explicit MPC to pure MATLAB code
% --------- Start Modifying Code Here -----------
file_name = 'empc_in_C';    % do not need to include '.c'
function_to_export = 'primal';
empc.optimizer.toC(function_to_export,file_name)
% --------- End Modifying Code Here -----------


% The generated mex-file can be compiled via typing "mex scriptName_mex.c"
% Hint: Firstly install MATLAB-build-in C/C++ compiler:  MinGW-w64
% --------- Start Modifying Code Here -----------
mex empc_in_C_mex.c   
% and evaluated inside Matlab
Uopt = empc_in_C_mex(x0)
% --------- End Modifying Code Here -----------

%% Task 6: Export to Python
% Syntax: 
%           empc.optimizer.toPython(filename, function, tiebreak)
% Inputs:
% empc.optimizer: single PolyUnion or an array thereof
%  filename: name of exported file (the '.py' extension will be added)
%  function: name of function to be exported
%  tiebreak: name of function to use to resolve tiebreaks
%            (use 'first-region' to break the sequential search once
%            the first region containing a given point is found)
empc.optimizer.toPython('empc_in_Python','primal','first-region')



%% Task 7:Export of online MPC:
% Export matrices of the optimisation problem via using function 
% "mpc.getMatrices()". Evaluate the MPC policy for an initial 
% condition x(t) = x_0 to obtain open-loop sequence of optimal control 
% actions U(t) = [u_0 u_1 ... u_{N-1}]'. Then analyse if all 
% constraints GU(t) <= W + Ex(t) are satisfied if such control action 
% will be used. Use x_0 = [2 0]' and x_0 = [-5 3]'.

% Some Information:
% Since we have used quadratic penalty x'*Q*x + u'*R*u, then our MPC leads
% to a QP problem formulated as
%   min_U 1/2 U'*H*U + x'*F*U + Cf*U + x'*Y*x + Cx*x + Cc
%    s.t. G*U <= W + E*x
%
% Here, U is the open-loop sequence of optimal control inputs
% and "x" is the vector of parameters.
%
% If the "dense" output is requested, the original primal
% optimizer is recovered by
%   zopt = Matrices.getback.S1*U + Matrices.getback.S2*x + Matrices.getback.S3
%   zopt = zopt(Matrices.requested_variables)

% Create online MPC and obtian matrices of the optimisation problem
N = 10;
mpc = MPCController(model, N);    % make sure that N is long (e.g. N = 10)
[Matrices] = mpc.getMatrices()

% --------- Start Modifying Code Here -----------
x0 = [2;0];
[u, isfeasible, info] = mpc.evaluate(x0);  % solve mpc for x0
U = info.U;                                % retrive open-loop sequence U
% Verify if all constraints G*U <= W + E*x hold for x_0 = [2 0]':
if all(Matrices.G*U' <= Matrices.W + Matrices.E*x0)
    disp('Feasible initial condition (all cons are satisfied)')
else
    disp('Infeasible initial condition (not all cons are satisfied)')
end

x0 = [-5;3];
[u, isfeasible, info] = mpc.evaluate(x0);  % solve mpc for x0
U = info.U;                                % retrive open-loop sequence U
% Verify if all constraints hold G*U <= W + E*x hold for x_0 = [-5 3]':
if all(Matrices.G*U' <= Matrices.W + Matrices.E*x0)
    disp('Feasible initial condition (all cons are satisfied)')
else
    disp('Infeasible initial condition (not all cons are satisfied)')
end
% --------- End Modifying Code Here -----------
