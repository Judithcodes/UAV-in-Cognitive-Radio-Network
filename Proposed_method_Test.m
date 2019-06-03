clear all, close all
clc
yalmip('clear');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - Parameters - %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H = 100; % height
x0 = -100; % initial location - x
y0 = 100*2; % initial location - y
xF = 500; % final location - x
yF = 100*2; % final location - y
x_cs = 0; % cognitive source location - x
y_cs = 0; % cognitive source location - y
x_cu = 300; % cognitive user location -x
y_cu = 0; % cognitive user location -y
x_e = 150; % eavesdropper location -x
y_e = 250; % evaesdropper location -y
x_pu = 300; % primary user location -x
y_pu = 250; % primary user location -y
Pcs_average = 10^(40/10-3)/2; % the average transmit power
Pcs_max = 10^(40/10-3); % the peak transmit power
Pu_average = (10^(4/10-3))/2; % the average jamming signal power
Pu_max = (10^(4/10-3)); % the peak jamming signal power
N = 200; % the number of time slots
T_flight = 100; % the UAV's trajectory versus period
V = 10; % the max velocity of the UAV
l = 0; % iteration index
noise = 10^(-104/10-3); % the power spectral density of the AWGN
beta = 10^(-14/10-3); % the channel power gain at the reference distance d = 1m
gamma = 10^8; % gamma = beta/noise = 90dB
phi = 3; % the path loss exponent
index = 0.5772156;
d_cs_cu = (sqrt((x_cs-x_cu)^2+(y_cs-y_cu)^2))^(-phi); % the distance between the cognitive source and the cognitive user
d_cs_e = (sqrt((x_cs-x_e)^2+(y_cs-y_e)^2))^(-phi); % the distance between the cognitive source and cognitive eavesdropper
d_cs_pu = (sqrt((x_cs-x_pu)^2+(y_cs-y_pu)^2))^(-phi); % the distance between the cognitive source and the primary user

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - Initial points - %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_initial = zeros(1,N); % the initial jamming trajectory - x                         
y_initial = zeros(1,N); % the initial  jamming trajectory - y
for i = 1:1:N
    x_initial(i) = x0+(i)*(xF-x0)/(N);
end
for i = 1:1:N
    y_initial(i) = y0;
end
%{
for i = 1:1:N
    if (i<=N/2)
        y_initial(i) = y0+(i)*(y_e-y0)/(N/2);
        x_initial(i) = x0+(i)*(xF-x0)/(N);
    else 
        y_initial(i) = y_e+(i)*(yF-y_e)/(N);
        x_initial(i) = x_e+(i)*(xF-x_e)/(N);
    end
end
%}
%{
Pcs_initial = Pcs_average*rand(1,N);                                                                            % the initial transmit power
%Pcs_initial = zeros(1,N);
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\PLS\Data\Pcs_initial.txt','w');
fprintf(fid, '%.32f \n',Pcs_initial);

Pu_initial = Pu_average*rand(1,N-20)*10^(-2+2)*2;                                                                   % the initial jamming signal power
a = Pu_average*rand(1,20)*10^(-5);
%a = zeros(1,20);
Pu_initial = [Pu_initial a];
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\PLS\Data\Pu_initial.txt','w');
fprintf(fid, '%.32f \n',Pu_initial);
%}

Pcs_initial = Pcs_average*rand(1,N-130);
a = Pcs_average*rand(1,80)*10^(-2);
b = Pcs_average*rand(1,50)*10^(-2);
Pcs_initial = [b Pcs_initial a];
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\Pcs_initial.txt','w');
fprintf(fid, '%.32f \n',Pcs_initial);

Pu_initial = Pu_average*rand(1,N-80)*2*10^(0);% the initial jamming signal power
a = Pu_average*rand(1,80)*10^(0)*10^(-2);% *10^(0)
b = Pu_average*rand(1,50)*10^(0)*10^(-2);% *10^(0)
Pu_initial = [Pu_initial a];
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\Pu_initial.txt','w');
fprintf(fid, '%.32f \n',Pu_initial);

% - the initial slack variables - %
z_initial = (((x_cu-x_initial).^2+(y_initial).^2+H^2));
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\z_initial.txt','w');
fprintf(fid, '%.32f \n',z_initial);
t_initial = (gamma*d_cs_cu*Pcs_initial.*z_initial./(gamma*Pu_initial+z_initial));
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\t_initial.txt','w');
fprintf(fid, '%.32f \n',t_initial);
k_initial = ((x_e-x_initial).^2+(y_e-y_initial).^2+H^2);
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\k_initial.txt','w');
fprintf(fid, '%.32f \n',k_initial);
u_initial = (gamma*d_cs_e.*Pcs_initial.*k_initial./(gamma*Pu_initial+k_initial));
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\u_initial.txt','w');
fprintf(fid, '%.32f \n',u_initial);
t_0_initial = (1./t_initial);
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\t_0_initial.txt','w');
fprintf(fid, '%.32f \n',t_0_initial);
v_initial = gamma*Pu_initial./k_initial;
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\v_initial.txt','w');
fprintf(fid, '%.32f \n',v_initial);
w_initial = 1./(v_initial+1);
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\w_initial.txt','w');
fprintf(fid, '%.32f \n',w_initial);

q_initial = ((x_pu-x_initial).^2+(y_pu-y_initial).^2+H^2);
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\q_initial.txt','w');
fprintf(fid, '%.32f \n',q_initial);
h_initial = 1./q_initial;
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\h_initial.txt','w');
fprintf(fid, '%.32f \n',h_initial);
%{
while (1)
%for l = 1:1:1
    % Define variables
    Pu = sdpvar(1,N,'full');
    Pcs = sdpvar(1,N,'full');
    x = sdpvar(1,N,'full');
    y = sdpvar(1,N,'full');
    
    z = sdpvar(1,N,'full');
    t = sdpvar(1,N,'full');
    d = sdpvar(1,N,'full');
    t_0 = sdpvar(1,N,'full');
    
    e = sdpvar(1,N,'full');
    k = sdpvar(1,N,'full');
    u = sdpvar(1,N,'full');
    
    v = sdpvar(1,N,'full');
    w = sdpvar(1,N,'full');
    c = sdpvar(1,N,'full');
    q = sdpvar(1,N,'full');
    h = sdpvar(1,N,'full');
    % Define an objective
    Objective = sum(d(1,1:N)-e(1,1:N));    
    % Define constraints 
    
    Constr = [];
    Constr = [Constr,
              cone([(x(1)-x0) (y(1)-y0)],T_flight*V/N),
              x(N)==xF,
              y(N)==yF];
    for n = 1:1:N-1
        Constr = [Constr,
                  cone([(x(n+1)-x(n)) (y(n+1)-y(n))],T_flight*V/N)];                                 
    end
    Constr = [Constr,
              (1/N)*sum(Pu(1,1:N)) <= Pu_average,
              (1/N)*sum(Pcs(1,1:N)) <= Pcs_average];
               
    for n = 1:1:N        
        A = (gamma*Pu_initial(n)+z_initial(n))/t_initial(n); 
        B = k_initial(n)/Pcs_initial(n);
        C = Pcs_initial(n)/w_initial(n);
        D = Pu_initial(n)/h_initial(n);
        Constr = [Constr,Pu(n) >= 0,
                  Pu(n) <= Pu_max,
                  Pcs(n) >= 0,
                  Pcs(n) <= Pcs_max,
                  
                  %%% - Rd constraints - %%%
                  z(n)<=(x_cu-x_initial(n))^2+2*(x_initial(n)-x_cu)*(x(n)-x_initial(n))+(y_initial(n))^2+2*y_initial(n)*(y(n)-y_initial(n))+H^2,
                  cone([t(n)*sqrt(0.5*A) (gamma*Pu(n)+z(n))*sqrt(0.5/A) (Pcs(n)-z(n))*(sqrt(gamma*d_cs_cu))/2],(Pcs(n)+z(n))*(sqrt(gamma*d_cs_cu))/2),
                  d(n)<=(log2(1+1/t_0_initial(n)))-(t_0(n)-t_0_initial(n))/((t_0_initial(n)*(t_0_initial(n)+1)*log(2))),
                  cone([1 0.5*(t(n)-t_0(n))],0.5*(t(n)+t_0(n))),
                  
                  %%% - Re constraints - %%%
                  cone([(x_e-x(n)) (y_e-y(n)) H (1/2)*(k(n)-1)],(1/2)*(k(n)+1)); % k(n) >= (x_e-x(n))^2+(y_e-y(n))^2+H^2,
                  cone([w(n)*sqrt(gamma*0.5*d_cs_e*C) Pcs(n)*sqrt(gamma*0.5*d_cs_e/C) 0.5*(u(n)-1)],0.5*(u(n)+1));%%%%problem
                  cone([1 0.5*(w(n)-v(n)-1)],0.5*(w(n)+v(n)+1));
                  cone([v(n)*sqrt(0.5*B) k(n)*sqrt(0.5/B) 0.5*(Pu(n)-1)*sqrt(gamma)],0.5*(Pu(n)+1)*sqrt(gamma));
                  e(n) >= log2(1+u_initial(n))+(u(n)-u_initial(n))/((1+u_initial(n))*log(2));
                  
                  %%% - Cognitive radio constraints - %%%
                  q(n) <= (x_pu-x_initial(n))^2+2*(x_initial(n)-x_pu)*(x(n)-x_initial(n))+(y_pu-y_initial(n))^2+2*(y_initial(n)-y_pu)*(y(n)-y_initial(n)); 
                  cone([h(n)*sqrt(0.5*beta*D) Pu(n)*sqrt(0.5*beta/D) 0.5*(c(n)-1)],0.5*(c(n)+1));
                  cone([1 0.5*(h(n)-q(n))],0.5*(h(n)+q(n)));
                
                  %%% - Implicit constraints - %%%
                  Pcs(n)+z(n) >= 0; % Rd constraint
                  t_0(n) > 0; % Rd constraint
                  t(n)+t_0(n) > 0; % Rd constraint
                  k(n) >= H^2; % Re constraint
                  v(n) > 0; % Re constraint
                  w(n) > 0; % Re constraint
                  w(n)+v(n)+1 > 0]; % Re constraint                     
    end                    
    % Set some options for YALMIP and solver
    option = sdpsettings('solver','sedumi','verbose',1);%
    % Solve the problem
    sol = solvesdp(Constr,-Objective, option);% 
    hold on
    plot(double(x_initial),double(y_initial));
    delta = double(x) - x_initial;
    eta = double(y) - y_initial;
    x_initial = double(x);
    y_initial = double(y);
    t_0_initial = double(t_0);
    t_initial = double(t);
    Pcs_initial = double(Pcs);
    Pu_initial = double(Pu);
    z_initial = double(z);
    k_initial = double(k);
    u_initial = double(u);
    %{
    disp('Gia tri x_initial:');
    disp(double(x_initial));
    disp('Gia tri y_initial:');
    disp(double(y_initial));
    disp('Gia tri t_0_initial:');
    disp(double(t_0_initial));
    disp('Gia tri t_initial:');
    disp(double(t_initial));
    disp('Gia tri Pcs_initial:');
    disp(double(Pcs_initial));
    disp('Gia tri Pu_initial:');
    disp(double(Pu_initial));
    disp('Gia tri z_initial:');
    disp(double(z_initial));
    disp('Gia tri k_initial:');
    disp(double(k_initial));
    disp('Gia tri u_initial:');
    disp(double(u_initial));
    %}
    disp('value');
    disp(double(sum(d(1,1:N)-e(1,1:N))));%
    %disp(double(Objective));
    disp(norm(double(delta)));
    disp(norm(double(eta)));
    %hold off
    disp(l);
    l = l + 1;
    %if (norm(double(delta)) <= 10^-2) && (norm(double(eta)) <= 10^-2)
    %if (double(sum(d(1,1:N)))>0) &(length(sol.info) >= length('Numerical problems (SeDuMi-1.3)'))%'Successfully solved (SeDuMi-1.3)') % 
    %    break;
    %end
end
%}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a = zeros(1,N);
 a(1)=1;
 for i = 2:1:N
     a(i)= a(i-1)+1;
 end
%R_new=double(sum(d(1,1:N)-e(1,1:N)));
cvx_solver SDPT3
R_cvx_update = zeros(1,N);
while (1)
    A = (gamma*Pu_initial+z_initial)./t_initial; 
    B = k_initial./v_initial;
    C = Pcs_initial./w_initial;
    D = Pu_initial./h_initial;
    %R_old=R_new;
    cvx_begin
        variables R_cvx(1,N) Pu_cvx(1,N) Pcs_cvx(1,N) x_cvx(1,N) y_cvx(1,N) 
        variables z_cvx(1,N) t_cvx(1,N) d_cvx(1,N) t_0_cvx(1,N) e_cvx(1,N) k_cvx(1,N) u_cvx(1,N) v_cvx(1,N) w_cvx(1,N) c_cvx(1,N) q_cvx(1,N) h_cvx(1,N);    
        maximize(sum(d_cvx(1,1:N)-e_cvx(1,1:N))/N) %
        subject to
            %%% - Trajectory constraints - %%%
            norm([(x_cvx(1)-x0) (y_cvx(1)-y0)])<=T_flight*V/N;
            for n = 1:1:N-1
                norm([(x_cvx(n+1)-x_cvx(n)) (y_cvx(n+1)-y_cvx(n))])<=T_flight*V/N;
            end
            x_cvx(N)==xF;
            y_cvx(N)==yF;
      
            %%% - Power constraints - %%%
            (1/N)*sum(Pu_cvx(1,1:N))<= Pu_average;
            (1/N)*sum(Pcs_cvx(1,1:N))<= Pcs_average;
            
            %%% - Cognitive radio constraint - %%%
            (1/N)*sum(beta*d_cs_pu*Pcs_cvx(1,1:N)+c_cvx(1,1:N))<=10^(-4);
            
            for n = 1:1:N
                %R_cvx(n)<=d_cvx(n)-e_cvx(n);
                %%% - Power constraints - %%%
                Pu_cvx(n) >= 0;
                Pu_cvx(n) <= Pu_max;
                Pcs_cvx(n) >= 0;
                Pcs_cvx(n) <= Pcs_max;
                
                %%% - Rd constraints - %%%
                z_cvx(n) <= (x_cu-x_initial(n))^2+2*(x_initial(n)-x_cu)*(x_cvx(n)-x_initial(n))+(y_initial(n))^2+2*y_initial(n)*(y_cvx(n)-y_initial(n))+H^2;
                norm([t_cvx(n)*sqrt(0.5*A(n)) (gamma*Pu_cvx(n)+z_cvx(n))*sqrt(0.5/A(n)) (Pcs_cvx(n)-z_cvx(n))*(sqrt(exp(-index)*gamma*d_cs_cu))/2])... 
                <= (Pcs_cvx(n)+z_cvx(n))*(sqrt(exp(-index)*gamma*d_cs_cu))/2;%%%Add Euler constant
                d_cvx(n) <= (log2(1+1/t_0_initial(n)))-(t_0_cvx(n)-t_0_initial(n))/((t_0_initial(n)*(t_0_initial(n)+1)*log(2)));
                norm([1 0.5*(t_cvx(n)-t_0_cvx(n))]) <= 0.5*(t_cvx(n)+t_0_cvx(n));
                
                %%% - Re constraints - %%%
                norm([(-x_e+x_cvx(n)) (-y_e+y_cvx(n)) H (1/2)*(k_cvx(n)-1)]) <= (1/2)*(k_cvx(n)+1); % k(n) >= (x_e-x(n))^2+(y_e-y(n))^2+H^2,
                norm([w_cvx(n)*sqrt(gamma*0.5*d_cs_e*C(n)) Pcs_cvx(n)*sqrt(gamma*0.5*d_cs_e/C(n)) 0.5*(u_cvx(n)-1)]) <= 0.5*(u_cvx(n)+1);%%%%problem
                norm([1 0.5*(w_cvx(n)-v_cvx(n)-1)]) <= 0.5*(w_cvx(n)+v_cvx(n)+1);
                norm([v_cvx(n)*sqrt(0.5*B(n)) k_cvx(n)*sqrt(0.5/B(n)) 0.5*(Pu_cvx(n)-1)*sqrt(gamma)]) <= 0.5*(Pu_cvx(n)+1)*sqrt(gamma);%%%%%%%%%%% problem
                e_cvx(n) >= log2(1+u_initial(n))+(u_cvx(n)-u_initial(n))/((1+u_initial(n))*log(2));
                
                %%% - Cognitive radio constraints - %%%
                q_cvx(n) <= (x_pu-x_initial(n))^2+2*(x_initial(n)-x_pu)*(x_cvx(n)-x_initial(n))+(y_pu-y_initial(n))^2+2*(y_initial(n)-y_pu)*(y_cvx(n)-y_initial(n)); 
                norm([h_cvx(n)*sqrt(0.5*beta*D(n)) Pu_cvx(n)*sqrt(0.5*beta/D(n)) 0.5*(c_cvx(n)-1)]) <= 0.5*(c_cvx(n)+1);
                norm([1 0.5*(h_cvx(n)-q_cvx(n))]) <= 0.5*(h_cvx(n)+q_cvx(n));
                
                %%% - Implicit constraints - %%%
                Pcs_cvx(n)+z_cvx(n) > 0; % Rd constraint
                t_0_cvx(n) > 0; % Rd constraint
                t_cvx(n)+t_0_cvx(n) > 0; % Rd constraint
                k_cvx(n) > H^2; % Re constraint
                v_cvx(n) > 0; % Re constraint
                w_cvx(n) > 0; % Re constraint
                w_cvx(n)+v_cvx(n)+1 > 0; % Re constraint
                Pu_cvx(n)+1 > 1;
                h_cvx(n)+q_cvx(n) > 0;
                c_cvx(n)+1 > 1;
                
            end
    cvx_end
    %app = log2((gamma*d_cs_e*Pcs_cvx./(1+gamma*Pu_cvx./k_cvx))+1);
    %disp(e_cvx - log2(u_cvx + 1));
    
    %%% - Plot - %%%
    figure(1)
    hold on
    plot(x_initial,y_initial);
    title('Trajectories of the UAV')
    xlabel('x(m)')
    ylabel('y(m)')
    
    %%% - Update - %%%
    delta_cvx = x_cvx - x_initial;
    eta_cvx = y_cvx - y_initial;
    x_initial = x_cvx;
    y_initial = y_cvx;
    t_0_initial = t_0_cvx;
    t_initial = t_cvx;
    Pcs_initial = Pcs_cvx;
    Pu_initial = Pu_cvx;
    z_initial = z_cvx;
    k_initial = k_cvx;
    u_initial = u_cvx;
    v_initial = v_cvx;
    w_initial = w_cvx;
    h_initial = h_cvx;
    %{
    disp('Gia tri x_initial:');
    disp(double(x_initial));
    disp('Gia tri y_initial:');
    disp(double(y_initial));
    disp('Gia tri t_0_initial:');
    disp(double(t_0_initial));
    disp('Gia tri t_initial:');
    disp(double(t_initial));
    disp('Gia tri Pcs_initial:');
    disp(double(Pcs_initial));
    disp('Gia tri Pu_initial:');
    disp(double(Pu_initial));
    disp('Gia tri z_initial:');
    disp(double(z_initial));
    disp('Gia tri k_initial:');
    disp(double(k_initial));
    disp('Gia tri u_initial:');
    disp(double(u_initial));
    %}
    
    uav_destination = (x_cu-x_initial).^2+(y_cu-y_initial).^2+H^2;
    uav_eavesdropper = (x_e-x_initial).^2+(y_e-y_initial).^2+H^2;
    rate_d = log2(1 + exp(-index)*gamma*d_cs_cu*Pcs_initial./(gamma*Pu_initial./uav_destination + 1));
    rate_e = log2(1 + gamma*d_cs_e*Pcs_initial./(gamma*Pu_initial./uav_eavesdropper + 1));
    rate_s = rate_d - rate_e;
    rate_average = (1/N)*sum(rate_s(1,1:N));
    
    rate_s_new = zeros(1,N);
    for r = 1:1:N
        if (rate_s(1,r)>=0)
            rate_s_new(1,r) = rate_s(1,r);
        else
            rate_s_new(1,r) = 0;
        end
    end
    Secrecy_rate = (1/N)*sum(rate_s_new(1,1:N));
    
    figure(2)
    hold on
    subplot(4,1,1)
    plot(a(1,1:N),Pu_initial(1,1:N));
    title('Power of the UAV')
    xlabel('slot')
    ylabel('Power')
    
    subplot(4,1,2)
    plot(a(1,1:N),((x_initial-x_e).^2+(y_initial-y_e).^2).^(1/2));
    title('distance between the UAV an eavesdropper')
    xlabel('slot')
    ylabel('Distance')
    
    subplot(4,1,3)
    plot(a(1,1:N),Pcs_initial(1,1:N));
    title('Power of the cognitive radio source')
    xlabel('slot')
    ylabel('Power')
    
    subplot(4,1,4)
    plot(a(1,1:N),rate_s_new);%d_cvx-e_cvx
    title('Rate')
    xlabel('slot')
    ylabel('Rate')
    
    %{
    disp('Gia tri d_cvx');
    disp(d_cvx);
    disp('Gia tri Rd');
    disp(rate_d);
    
    disp('Gia tri e_cvx');
    disp(e_cvx);
    disp('Gia tri Re');
    disp(rate_e);
    %}
    
    %{
    disp('Rd - d');
    disp(rate_d - d_cvx);
    disp('Re - e');
    disp(rate_e - e_cvx);
    %disp(app - rate_e);
    %}
    
    %disp(sum(Pu_initial(1,1:N))/N);
    disp('Approximated value');
    disp(sum(d_cvx(1,1:N)-e_cvx(1,1:N))/N); %disp(double(Objective))
    disp('Original value');
    disp(rate_average);
    disp('Secrecy rate');
    disp(Secrecy_rate);
    
    disp('Iteration');
    disp(l);
    l = l + 1;
    
    R_cvx = d_cvx-e_cvx;
    
    disp('Step');
    disp(sum(R_cvx(1,1:N))/N-sum(R_cvx_update(1,1:N))/N);
    
    %disp('The power of UAV')
    %disp(double(Pu_initial));
    
    %disp('The power of cognitive source');
    %disp(double(Pcs_initial));
    
    if (abs(sum(R_cvx(1,1:N))/N-sum(R_cvx_update(1,1:N))/N)<=10^-6)
        break;
    end
    R_cvx_update = R_cvx;
end










