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
x_pu = 0; % primary user location -x
y_pu = 250; % primary user location -y
Pcs_average = 10^(40/10-3)/2; % the average transmit power
Pcs_max = 10^(40/10-3); % the peak transmit power
Pu_average = (10^(4/10-3))/2*10^0; % the average jamming signal power
Pu_max = (10^(4/10-3))*10^0; % the peak jamming signal power
N = 200; % the number of time slots
T_flight = 500; % the UAV's trajectory versus period
V = 20/2; % the max velocity of the UAV
l = 0; % iteration index
noise = 10^(-70/10-3); % the power spectral density of the AWGN
beta = 10^(10/10-3); % the channel power gain at the reference distance d = 1m
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


Pcs_initial = Pcs_average*rand(1,N);                                                                            % the initial transmit power
%Pcs_initial = zeros(1,N);
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\Pcs_initial.txt','w');
fprintf(fid, '%.32f \n',Pcs_initial);

Pu_initial = Pu_average*rand(1,N-20)*10^(-2+2)*2;                                                                   % the initial jamming signal power
a = Pu_average*rand(1,20)*10^(-5);
%a = zeros(1,20);
Pu_initial = [Pu_initial a];
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Data\Pu_initial.txt','w');
fprintf(fid, '%.32f \n',Pu_initial);

%{
Pcs_initial = Pcs_average*rand(1,N-130);
a = Pcs_average*rand(1,80)*10^(-2);
b = Pcs_average*rand(1,50)*10^(-2);
Pcs_initial = [b Pcs_initial a];
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\PLS\Data\Pcs_initial.txt','w');
fprintf(fid, '%.32f \n',Pcs_initial);

Pu_initial = Pu_average*rand(1,N-80)*2*10^(0);% the initial jamming signal power
a = Pu_average*rand(1,80)*10^(0)*10^(-2);% *10^(0)
b = Pu_average*rand(1,50)*10^(0)*10^(-2);% *10^(0)
Pu_initial = [Pu_initial a];
fid = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\PLS\Data\Pu_initial.txt','w');
fprintf(fid, '%.32f \n',Pu_initial);
%}

% - the initial slack variables - %
z_initial = (((x_cu-x_initial).^2+(y_initial).^2+H^2));

t_initial = (exp(-index)*gamma*d_cs_cu*Pcs_initial.*z_initial./(gamma*Pu_initial+z_initial));

k_initial = ((x_e-x_initial).^2+(y_e-y_initial).^2+H^2);

u_initial = (gamma*d_cs_e.*Pcs_initial.*k_initial./(gamma*Pu_initial+k_initial));

t_0_initial = (1./t_initial);

v_initial = gamma*Pu_initial./k_initial;

w_initial = 1./(v_initial+1);

q_initial = ((x_pu-x_initial).^2+(y_pu-y_initial).^2+H^2);

h_initial = 1./q_initial;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 a = zeros(1,N);
 a(1)=1;
 for i = 2:1:N
     a(i)= a(i-1)+1;
 end
%R_new=double(sum(d(1,1:N)-e(1,1:N)));
cvx_solver SDPT3
scale_factor = 10^5;
R_cvx_update = zeros(1,N);
while (1)
    A = (gamma*Pu_initial+z_initial)./t_initial; 
    B = k_initial./v_initial;
    C = Pcs_initial./w_initial;
    D = Pu_initial./h_initial;
    %R_old=R_new;
    cvx_begin
        variables R_cvx(1,N) Pu_cvx(1,N) Pcs_cvx(1,N) x_cvx(1,N) %y_cvx(1,N) 
        variables z_cvx(1,N) t_cvx(1,N) d_cvx(1,N) t_0_cvx(1,N) e_cvx(1,N) k_cvx(1,N) u_cvx(1,N) v_cvx(1,N) w_cvx(1,N) c_cvx(1,N) q_cvx(1,N) h_cvx(1,N);    
        maximize(sum(d_cvx(1,1:N)-e_cvx(1,1:N))/N) %
        subject to
            %%% - Trajectory constraints - %%%
            (x_cvx(1)-x0) <= T_flight*V/N;%norm([(x_cvx(1)-x0) (y_cvx(1)-y0)]) <= T_flight*V/N;
            (x_cvx(1)-x0) >= -T_flight*V/N;
            for n = 1:1:N-1
                (x_cvx(n+1)-x_cvx(n)) <= T_flight*V/N;%norm([(x_cvx(n+1)-x_cvx(n)) (y_cvx(n+1)-y_cvx(n))]) <= T_flight*V/N;
                (x_cvx(n+1)-x_cvx(n)) >= -T_flight*V/N;
            end
            x_cvx(N) == xF;
            %y_cvx(N) == yF;
      
            %%% - Power constraints - %%%
            (1/N)*sum(Pu_cvx(1,1:N)) <= Pu_average;
            (1/N)*sum(Pcs_cvx(1,1:N)) <= Pcs_average;
            
            %%% - Cognitive radio constraint - %%%
            (1/N)*sum(beta*d_cs_pu*Pcs_cvx(1,1:N)+c_cvx(1,1:N))<=10^(-7);
            
            for n = 1:1:N
                %y_cvx(n) == y0;
                %%% - Power constraints - %%%
                Pu_cvx(n) >= 0;
                Pu_cvx(n) <= Pu_max;
                Pcs_cvx(n) >= 0;
                Pcs_cvx(n) <= Pcs_max;
                
                %%% - Rd constraints - %%%
                scale_factor*z_cvx(n) <= scale_factor*((x_cu - x_initial(n))^2 + 2*(x_initial(n)-x_cu)*(x_cvx(n)-x_initial(n)) + (y_initial(n))^2 + H^2);
                scale_factor*norm([t_cvx(n)*sqrt(0.5*A(n)) (gamma*Pu_cvx(n)+z_cvx(n))*sqrt(0.5/A(n)) (Pcs_cvx(n)-z_cvx(n))*(sqrt(exp(-index)*gamma*d_cs_cu))/2])... 
                <= scale_factor*(Pcs_cvx(n)+z_cvx(n))*(sqrt(exp(-index)*gamma*d_cs_cu))/2;%%%Add Euler constant
                scale_factor*d_cvx(n) <= scale_factor*((log2(1+1/t_0_initial(n)))-(t_0_cvx(n)-t_0_initial(n))/((t_0_initial(n)*(t_0_initial(n)+1)*log(2))));
                scale_factor*norm([1 0.5*(t_cvx(n)-t_0_cvx(n))]) <= scale_factor*0.5*(t_cvx(n)+t_0_cvx(n));
                
                %%% - Re constraints - %%%
                scale_factor*norm([(-x_e+x_cvx(n)) (-y_e+y_initial(n)) H (1/2)*(k_cvx(n)-1)]) <= scale_factor*(1/2)*(k_cvx(n)+1); % k(n) >= (x_e-x(n))^2+(y_e-y(n))^2+H^2,
                scale_factor*norm([w_cvx(n)*sqrt(gamma*0.5*d_cs_e*C(n)) Pcs_cvx(n)*sqrt(gamma*0.5*d_cs_e/C(n)) 0.5*(u_cvx(n)-1)]) <= scale_factor*0.5*(u_cvx(n)+1);%%%%problem
                scale_factor*norm([1 0.5*(w_cvx(n)-v_cvx(n)-1)]) <= scale_factor*0.5*(w_cvx(n)+v_cvx(n)+1);
                scale_factor*norm([v_cvx(n)*sqrt(0.5*B(n)) k_cvx(n)*sqrt(0.5/B(n)) 0.5*(Pu_cvx(n)-1)*sqrt(gamma)]) <= scale_factor*0.5*(Pu_cvx(n)+1)*sqrt(gamma);%%%%%%%%%%% problem
                scale_factor*e_cvx(n) >= scale_factor*((log2(1+u_initial(n))+(u_cvx(n)-u_initial(n))/((1+u_initial(n))*log(2))));
                
                %%% - Cognitive radio constraints - %%%
                scale_factor*q_cvx(n) <= scale_factor*((x_pu-x_initial(n))^2+2*(x_initial(n)-x_pu)*(x_cvx(n)-x_initial(n))+(y_pu-y_initial(n))^2); 
                scale_factor*norm([h_cvx(n)*sqrt(0.5*beta*D(n)) Pu_cvx(n)*sqrt(0.5*beta/D(n)) 0.5*(c_cvx(n)-1)]) <= scale_factor*0.5*(c_cvx(n)+1);
                scale_factor*norm([1 0.5*(h_cvx(n)-q_cvx(n))]) <= scale_factor*0.5*(h_cvx(n)+q_cvx(n));
                
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
    %eta_cvx = y_cvx - y_initial;
    x_initial = x_cvx;
    %y_initial = y_cvx;
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
    subplot(3,1,1)
    plot(a(1,1:N),Pu_initial(1,1:N));
    title('Power of the UAV')
    xlabel('slot')
    ylabel('Power')
    
    subplot(3,1,2)
    plot(a(1,1:N),((x_initial-x_e).^2+(y_initial-y_e).^2).^(1/2));
    title('distance between the UAV an eavesdropper')
    xlabel('slot')
    ylabel('Distance')
    
    subplot(3,1,3)
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
    
    %{
    disp('Pu_cvx');
    disp(Pu_cvx);
    disp('Pcs_cvx');
    disp(Pcs_cvx);
    disp('x_cvx');
    disp(x_cvx);
    disp('y_cvx');
    disp(y_cvx);
    disp('z_cvx');
    disp(z_cvx);
    disp('t_cvx');
    disp(t_cvx);
    disp('d_cvx');
    disp(d_cvx);
    disp('t_0_cvx');
    disp(t_0_cvx);
    disp('e_cvx');
    disp(e_cvx);
    disp('k_cvx');
    disp(k_cvx);
    disp('u_cvx');
    disp(u_cvx);
    disp('v_cvx');
    disp(v_cvx);
    disp('w_cvx');
    disp(w_cvx);
    disp('c_cvx');
    disp(c_cvx);
    disp('q_cvx');
    disp(q_cvx);
    disp('h_cvx');
    disp(h_cvx);
    %}
    
    disp('Interfere Power');
    disp((1/N)*sum(beta*d_cs_pu*Pcs_cvx(1,1:N)+c_cvx(1,1:N)));
    if (abs(sum(R_cvx(1,1:N))/N-sum(R_cvx_update(1,1:N))/N)<=10^-6)
        break;
    end
    R_cvx_update = R_cvx;
end











