%% =========================================================================
% Scenario:
%       Two CAV is leading the motion of n HDVs behind and is also
%       following one head vehicle
%       One sudden disturbance happens at one HDV behind the CAV
%       Title: Safety-Critical Traffic Control by Connected Automated Vehicles
%       We test the nominal controller with the STC under the 
%           Senerio 1: Head vehicel decelerates,
%           Senerio 2: Head vehicel accelarate.


clc;clear; close all;
addpath('..\_model');



% -------------------------------------------------------------------------
%   Parameter setup
%--------------------------------------------------------------------------
m = 0;                  % number of preceding vehicles
n = 5;                  % number of following vehicles
PerturbedID = 0;        % perturbation on vehicle
                        % 0. Head vehicle
                        % 1 - m. Preceding vehicles
                        % m+2 - n+m+1. Following vehicles
PerturbedType = 2;      % perturbation type
                        % 1:trigonometric ;  2: Braking then accelarate
                        % 3.reference speed decrease to another constant
showtheresults = 2;                 % 1: velocity 2. accelerate 3. space
mix = 1;                % Mix traffic or all HDVs
reference = 1;          % reference signal

% ------------------------------------------
% Connectivity pattern
% ------------------------------------------
connectivityType = 2;               % Different connectivity patterns

controllertype = 1;                 %  Different controller 1:STC 2:nominal
feedback = 2;                      %  Different gain setting

switch feedback
    case 1
K= [0,0,0,0,1,-1,1,-1,1,-1,1,-1];
   case 2
K= [0,0,0,0,-2,0.2,-2,0.2,-2,0.2,-2,0.2];    % Feedback gain
end

switch connectivityType
    case 1
        K(3:end) = 0;
    case 2
        K(5:end) = 0;
    case 3 
        K(9:end) = 0;
    case 4
        K(13:end) = 0;
        %
end

switch reference 
     case 1
        C = 0
     case 2
        C = 0.3   
     case 3
        C = 1
     case 4
        C = 3
end
% ------------------------------------------
% Parameters in the car-following model
% ------------------------------------------
alpha = 0.6; % Driver Model: OVM
beta  = 0.9;
s_st  = 5;
s_go  = 35;

% Traffic equilibrium
v_star   = 20;   % Equilibrium velocity
acel_max = 7;
dcel_max = -7;
v_max    = 40;
s_star   = acos(1-v_star/v_max*2)/pi*(s_go-s_st)+s_st; % Equilibrium spacing

% linearized model
alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;

% Simulation length
TotalTime = 200;
Tstep     = 0.1;
NumStep   = TotalTime/Tstep;

% ------------------------------------------------------------------------
% Some output information
% ------------------------------------------------------------------------
fprintf('============================================================\n')
fprintf('    Traffic Simulation when One Perturbation Happens Ahead \n')
fprintf('          By Guanbo \n')
fprintf('============================================================\n')

fprintf('Number of HDV vehicles behind: %d\n',n)
fprintf('Number of HDV vehicles ahead : %d\n',m)
fprintf('Perturbation vehicle Id      : %d\n',PerturbedID)
fprintf('---------------------------\n')
fprintf('HDV car-following model: optimal velocity model (OVM) \n')
fprintf('Parameter setup in HDV car-following model: \n')
fprintf('    alpha  beta  s_st  s_go  v_max \n    %4.2f  %4.2f  %4.2f  %4.2f  %4.2f\n',alpha,beta,s_st,s_go,v_max)
fprintf('Coefficients in linearized HDV car-following model: \n')
fprintf('    alpha1  alpha2  alpha3 \n    %4.2f    %4.2f    %4.2f \n',alpha1,alpha2,alpha3)
fprintf('---------------------------\n')
fprintf('Feedback gain of the controller:\n')
fprintf('mu_{-2}  k_{-2}  mu_{-1}  k_{-1}  mu_{1}  k_{1}  mu_{2}  k_{2}\n')
 % fprintf('%4.2f      %4.2f   %4.2f    %4.2f    %4.2f    %4.2f\n',K(1),K(2),K(3),K(4),K(6),K(7),K(8),K(9))
fprintf('---------------------------\n')
fprintf('Simulation length (time step): %d  (%4.2f)\n',TotalTime,Tstep)  % this can be improved
fprintf('-----------------------------------------------------------\n')
fprintf('   Simulation beigns ...')

% ------------------------------------------------------------------------
% Traffic simulation
% ------------------------------------------------------------------------

switch mix
    case 1
        % When will the controller work. 0:Controller Work; Large: won't work
        ActuationTime = 0;
    case 0
        ActuationTime = 99999;
end

% -----------------------------------------------
% Define state variables
% -----------------------------------------------
%Initial State for each vehicle
S = zeros(NumStep,m+n+2,3);
dev_s = 0;
dev_v = 0;
co_v = 1.0;
v_ini = co_v*v_star; %Initial velocity
%from -dev to dev
S(1,:,1) = linspace(0,-(m+n+1)*s_star,m+n+2)'+(rand(m+n+2,1)*2*dev_s-dev_s);
%The vehicles are uniformly distributed on the ring road with a random deviation
S(1,:,2) = v_ini*ones(m+n+2,1)+(rand(m+n+2,1)*2*dev_v-dev_v);

% meaning of parameters
% 1:head vehicle
% 2~(m+1): preceding vehicles
% m+2:CAV
% (m+3)~(m+n+2): following vehicles
ID = zeros(1,m+n+2);
if mix
    ID(m+2) = 1;
end

X = zeros(2*(m+n+1),NumStep);
u_1 = zeros(NumStep,1);
u_2 = zeros(NumStep,1);
V_diff = zeros(NumStep,m+n+1);  %Velocity Difference
D_diff = zeros(NumStep,m+n+1);  %Following Distance

% ---------------------------------------------------------
% Simulation starts here
% ---------------------------------------------------------
tic
for k = 1:NumStep-1
    %Update acceleration
    V_diff(k,:) = S(k,1:(end-1),2)-S(k,2:end,2);
    D_diff(k,:) = S(k,1:(end-1),1)-S(k,2:end,1);
    cal_D = D_diff(k,:); %For the boundary of Optimal Veloicity Calculation
    for i = 1:m+n+1
        if cal_D(i)>s_go
            cal_D(i) = s_go;
        elseif cal_D(i)<s_st
            cal_D(i) = s_st;
        end
    end
    
    %OVM Model
    %V_d = v_max/2*(1-cos(pi*(h-h_st)/(h_go-h_st)));
    %a2 = alpha*(V_h-v2)+beta*(v1-v2);
    acel = alpha*(v_max/2*(1-cos(pi*(cal_D-s_st)/(s_go-s_st)))-S(k,2:end,2))+beta*V_diff(k,:);
    acel(acel>acel_max)=acel_max;
    acel(acel<dcel_max)=dcel_max;
    % SD as ADAS to prevent crash
    acel_sd = (S(k,2:end,2).^2-S(k,1:(end-1),2).^2)./2./D_diff(k,:);
    acel(acel_sd>abs(dcel_max)) = dcel_max;
    
    S(k,2:end,3) = acel;
    % the preceding vehicle
    S(k,1,3) = 0;
    
    % Perturbation
    switch PerturbedType
        case 1
            P_A = 0.2;
            P_T = 12;
            if k*Tstep>20 && k*Tstep<20+P_T
                S(k,PerturbedID+1,3)=P_A*cos(2*pi/P_T*(k*Tstep-20));
            end
        case 2
            if (k*Tstep>20)&&(k*Tstep<23.3)
                S(k,PerturbedID+1,3)=-6;
            elseif (k*Tstep>23.3)&&(k*Tstep<26.6)
                 S(k,PerturbedID+1,3)=6;
            end
        case 3
            if k*Tstep == 20
                S(k,PerturbedID+1,2) = v_star - 0.4;
                
            end
    end
    
    X(1:2:end,k) = reshape(D_diff(k,:),m+n+1,1) - s_star;
    X(2:2:end,k) = reshape(S(k,2:end,2),m+n+1,1) - v_star;
    
   
    
    
      %STC controller  prepareation 

            
           
 
            
        %prepareation for QP
            tau = 1;  
            gamma_0 = 10;
            gamma_1 = 10;
            gamma_2 = 10;
            gamma_3 = 10;
            gamma_4 = 10;
            gamma_5 = 10;
            
            p = 100;
           
           
            % set h_i = s_i-tao*v_i, refer to eq 37 in huanyu's essay  
            %h_error set as eq45 from huanyu
            partial_h_0_partial_x   = [1,-1*tau,0,0,0,0,0,0,0,0,0,0];                                                                                                                                                                                                                                                                                                                         
            partial_h_1_e_partial_x = [0,0,0,0,1,-1*tau,0,0,0,0,0,0]-partial_h_0_partial_x;
            partial_h_2_e_partial_x = [0,0,0,0,0,0,1,-1*tau,0,0,0,0]-partial_h_0_partial_x;
            partial_h_1_partial_x   = [0,0,1,-1*tau,0,0,0,0,0,0,0,0];
            partial_h_4_e_partial_x = [0,0,0,0,0,0,0,0,1,-1*tau,0,0]-partial_h_1_partial_x;
            partial_h_5_e_partial_x = [0,0,0,0,0,0,0,0,0,0,1,-1*tau]-partial_h_1_partial_x;
    
            switch controllertype
                case 1
       %STC Controller u design
          if k > ActuationTime/Tstep
             u_10 = K*X(:,k);
             u_20 = K*X(:,k);
                       %nonlinear dynamic model 
            N = m+n;
            f = zeros(2*N+2,1);
            g = zeros(2*N+2,2);
           
            
           for i=1:2
             f(2*i-1:2*i) = [S(k,i+1,2)-S(k,i,2)+C*sin(k*Tstep);0];
           end
           for i=3:6
             f(2*i-1:2*i) = [ S(k,i+1,2)-S(k,i,2); alpha*(v_max/2*(1-cos(pi*(S(k,i+1,1)-S(k,i,1)-s_st)/(s_go-s_st))) -S(k,i,2))+beta*(S(k,i+1,1)-S(k,i,1)) ];
            end
           
            
            g(2,1) = 1;
            g(4,2) = 1;
             
             
                % Objective function: (u_1-u_10)^2 + (u_2-u_20)^2 + p*sigma_1^2 + p*sigma_2^2  + p*sigma_3^2 p*sigma_4^2;
                % x=[u1, sigma1, sigma2, u2, sigma3, sigma 4];
                P = [2, 0, 0, 0, 0, 0;  
                    0, 2, 0, 0, 0, 0;
                    0   0  2*p, 0, 0, 0;
                    0, 0, 0 ,2*p, 0 ,0; 
                    0, 0, 0, 0, 2*p, 0;
                    0, 0, 0, 0, 0, 2*p];
                q = [-2 * u_10;  -2*u_20; 0; 0;  0; 0];
                %components of A and b 
                        L_f_h_0   = partial_h_0_partial_x  * f;
                        L_f_h_1   = partial_h_1_partial_x  * f;
                        L_f_h_e_1 = partial_h_1_e_partial_x* f;
                        L_f_h_e_2 = partial_h_2_e_partial_x* f;
                        L_f_h_e_4 = partial_h_4_e_partial_x* f;
                        L_f_h_e_5 = partial_h_5_e_partial_x* f;
                        
                        L_g_h_0   = partial_h_0_partial_x*g;
                        L_g_h_1   = partial_h_1_partial_x*g;
                        L_g_h_e_1 = partial_h_1_e_partial_x*g;
                        L_g_h_e_2 = partial_h_2_e_partial_x*g;
                        L_g_h_e_4 = partial_h_4_e_partial_x*g;
                        L_g_h_e_5 = partial_h_5_e_partial_x*g;

% eq37
                        h_0 = (S(k,m+1,1)-S(k,m+2,1)) - tau*S(k,m+2,2);
                        h_2 = (S(k,m+2,1)-S(k,m+3,1)) - tau*S(k,m+3,2);
                        h_3 = (S(k,m+3,1)-S(k,m+4,1)) - tau*S(k,m+4,2);
                        h_1 = (S(k,m+4,1)-S(k,m+5,1)) - tau*S(k,m+5,2);
                        h_4 = (S(k,m+5,1)-S(k,m+6,1)) - tau*S(k,m+6,2);
                        h_5 = (S(k,m+6,1)-S(k,m+7,1)) - tau*S(k,m+7,2);
                        
                        h_overline_1 = h_2 -  h_0;
                        h_overline_2 = h_3 -  h_0;
                        h_overline_4 = h_4 -  h_1;
                        h_overline_5 = h_5 -  h_1;
                     


            % Constraints: Ax <= b
                A_QP = [-L_g_h_0,     0, 0, 0 ,0;
                        -L_g_h_1,     0, 0, 0, 0;
                        -L_g_h_e_1,   -1, 0, 0, 0; 
                        -L_g_h_e_2,   0, -1, 0, 0
                        -L_g_h_e_4,   0, 0, -1, 0;
                        -L_g_h_e_5,   0, 0, 0, -1;
                        0, 0, -1, 0, 0, 0; 
                        0, 0, 0, -1, 0, 0; 
                        0, 0, 0, 0, -1, 0;       
                        0, 0, 0, 0, 0, -1 ;];
                b = [       L_f_h_0     + gamma_0*h_0; 
                            L_f_h_1     + gamma_1*h_1; 
                            L_f_h_e_1   + gamma_1*h_overline_1;
                            L_f_h_e_2   + gamma_2*h_overline_2 ;
                            L_f_h_e_4   + gamma_4*h_overline_4 ;
                            L_f_h_e_5   + gamma_5*h_overline_5 ;
                            0;
                            0;
                            0;
                            0; ];

      % Solve the quadratic program using quadprog
              options = optimset('Display', 'off');
              [x, fval, exitflag, output] = quadprog(P, q, A_QP, b, [], [], [], [], [], options);

      % Extract the optimal values
               u_1(k) = x(1);
               u_2(k) = x(2);
               sigma_1_optimal = x(3);
               sigma_2_optimal = x(4);
               sigma_3_optimal = x(5);
               sigma_4_optimal = x(6);
      % fprintf('Optimal u: %.4f\n', u(k));
      % fprintf('Optimal sigma_1: %.4f\n', sigma_1_optimal);
      % fprintf('Optimal sigma_2: %.4f\n', sigma_2_optimal);
       
       
      if u_1(k) > acel_max
            u_1(k) = acel_max;
      elseif u_1(k) < dcel_max
            u_1(k) = dcel_max;
      end
        S(k,m+2,3) = u_1(k) + S(k,m+2,3);
        
          
        if u_2(k) > acel_max
            u_2(k) = acel_max;
      elseif u_2(k) < dcel_max
            u_2(k) = dcel_max;
      end
        S(k,m+5,3) = u_2(k) + S(k,m+5,3);
        
        
          end
    
  case 2
        if k > ActuationTime/Tstep
             u_1(k) = K*X(:,k);
             u_2(k) = K*X(:,k);
    
      if u_1(k) > acel_max
            u_1(k) = acel_max;
      elseif u_1(k) < dcel_max
            u_1(k) = dcel_max;
      end
        S(k,m+2,3) = u_1(k) + S(k,m+2,3);
        
          
        if u_2(k) > acel_max
            u_2(k) = acel_max;
      elseif u_2(k) < dcel_max
            u_2(k) = dcel_max;
      end
        S(k,m+5,3) = u_2(k) + S(k,m+5,3);
             
             
        end
             
            end  
             
             
             
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    
    
end
tsim = toc;

fprintf('  ends at %6.4f seconds \n', tsim);

% ------------------------------------------------------------------------
%  Plot the results for velocvity
% ------------------------------------------------------------------------
fprintf('-----------------------------------------------------------\n')
fprintf('    Now plot the velocity profiles for demonstration, please wait ... \n')

Wsize = 22;
figure;



switch showtheresults
    
case 1 % velocity
            i = 1;
            p1 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[190 190 190]/255);
            hold on;
            for i=2:(m+1)
                p2 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[90 90 90]/255);
            end
            i = m+2;
            p3 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[244, 53, 124]/255);

            for i=(m+3):(m+4)
                p4 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[67, 121, 227]/255);
            end
            i = m+5;
               p5 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[244, 53, 124]/255);

            for i=(m+6):(n+m+2)
                p6 = plot(Tstep:Tstep:TotalTime,S(:,i,2),'-','linewidth',2,'Color',[67, 121, 227]/255);
            end
            set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-4);
            grid on;
            xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',Wsize,'Interpreter','latex','Color','k');
            yl = ylabel('Velocity [$\mathrm{m/s^2}$]','fontsize',Wsize,'Interpreter','latex','Color','k');
             
            set(gca,'xlim',[18,45]);
            
            switch PerturbedType
                case 1
                   set(gca,'ylim',[19,21]);
                case 2
                   set(gca,'ylim',[0,30]);
                case 3
                   set(gca,'ylim',[19,20.5]);
            end
         
case 2 %acceleration
            i = 1;
            p1 = plot(Tstep:Tstep:TotalTime,S(:,i,3),'-','linewidth',2,'Color',[190 190 190]/255);
            hold on;
            for i=2:(m+1)
                p2 = plot(Tstep:Tstep:TotalTime,S(:,i,3),'-','linewidth',2,'Color',[90 90 90]/255);
            end
            i = m+2;
               p3 = plot(Tstep:Tstep:TotalTime,S(:,i,3),'-','linewidth',2,'Color',[244, 53, 124]/255);

            for i=(m+3):(m+4)
                p4 = plot(Tstep:Tstep:TotalTime,S(:,i,3),'-','linewidth',2,'Color',[67, 121, 227]/255);
            end
            i = m+5;
               p5 = plot(Tstep:Tstep:TotalTime,S(:,i,3),'-','linewidth',2,'Color',[244, 53, 124]/255);

            for i=(m+6):(n+m+2)
                p6 = plot(Tstep:Tstep:TotalTime,S(:,i,3),'-','linewidth',2,'Color',[67, 121, 227]/255);
            end
            
            set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-4);
            grid on;
            xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',Wsize,'Interpreter','latex','Color','k');
            yl = ylabel('acceleration [$\mathrm{m/s^2}$]','fontsize',Wsize,'Interpreter','latex','Color','k');
             set(gca,'xlim',[18,40]);
          
             switch PerturbedType
                case 1
                   set(gca,'ylim',[-1,1]);
                case 2
                   set(gca,'ylim',[-8,8]);
                case 3
                   set(gca,'ylim',[-0.5,0.5]);
            end
            
            
case 3 %space
        i = 1;
        p1 = plot(Tstep:Tstep:TotalTime,S(:,i,1)-S(:,i+1,1)-20,'-','linewidth',2,'Color',[244, 53, 124]/255);
        hold on;
        for i=2:(m+3)
            p2 = plot(Tstep:Tstep:TotalTime,S(:,i,1)-S(:,i+1,1)-20,'-','linewidth',2,'Color',[67, 121, 227]/255);
        end

        i = m+4;
           p5 = plot(Tstep:Tstep:TotalTime,S(:,i,1)-S(:,i+1,1)-20,'-','linewidth',2,'Color',[244, 53, 124]/255);

        for i=(m+5):(n+m+1)
            p6 = plot(Tstep:Tstep:TotalTime,S(:,i,1)-S(:,i+1,1)-20,'-','linewidth',2,'Color',[67, 121, 227]/255);
        end
     
          set(gca,'TickLabelInterpreter','latex','fontsize',Wsize-4);
            grid on;
            xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',Wsize,'Interpreter','latex','Color','k');
            yl = ylabel('space [$\mathrm{m}$]','fontsize',Wsize,'Interpreter','latex','Color','k');
             set(gca,'xlim',[20,45]);
             switch PerturbedType
                case 1
                   set(gca,'ylim',[-1,1]);
                case 2
                   set(gca,'ylim',[-15,10]);
                case 3
                   set(gca,'ylim',[-1,1]);
            end
end











if m == 0
    l = legend([p1 p3 p4],'Head vehicle','CAV','HDVs behind','Location','NorthEast');
elseif n == 0
    
else
l = legend([p1 p2 p3 p4],'Head vehicle','HDVs ahead','CAV','HDVs behind','Location','NorthEast');
end

l.Position=[0.5,0.6,0.4,0.3];

set(gcf,'Position',[250 150 480 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';
set(l,'fontsize',Wsize-4,'box','off','Interpreter','latex');
% print(gcf,['..\Figures\Simulation_PerturbationAhead_Controller_',num2str(controllerType)],'-painters','-depsc2','-r300');


