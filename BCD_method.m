clear all, close all
clc
%yalmip('clear');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% - Parameters - %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h = 100; % height
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
x_pu = 300-300; % primary user location -x
y_pu = 250; % primary user location -y
Pcs_average = 10^(40/10-3)/2; % the average transmit power
Pcs_max = 10^(40/10-3); % the peak transmit power
Pu_average = (10^(4/10-3))/2; % the average jamming signal power
Pu_max = (10^(4/10-3)); % the peak jamming signal power
N = 200; % the number of time slots
T_flight = 100-30; % the UAV's trajectory versus period
V = 10; % the max velocity of the UAV
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

fileID = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Input_Proposed_method_200_1\Pcs_initial.txt');
[Pcs_initial,count] = fscanf(fileID,'%f');
fclose(fileID);
Pcs_initial = Pcs_initial'; % the initial transmit power

fileID = fopen('C:\Users\NGUYEN XUAN PHU\Desktop\Full_Project\Input_Proposed_method_200_1\Pu_initial.txt');
[Pu_initial,count] = fscanf(fileID,'%f');
fclose(fileID);
Pu_initial = Pu_initial'; % the initial jamming signal power

t_0_initial = 1./(Pcs_initial.*exp(-index)*gamma*d_cs_cu./(gamma*Pu_initial./((x_cu-x_initial).^2+(y_cu-y_initial).^2+h^2) + 1));
m_initial = gamma.*d_cs_e.*Pcs_initial./(gamma./((x_e - x_initial).^2 + (y_e - y_initial).^2 + h^2) + 1);

k_initial = (x_e-x_initial).^2 + (y_e-y_initial).^2 + h^2;
z_initial = (x_cu - x_initial).^2 + (y_cu - y_initial).^2 + h^2;
t_1_initial = gamma*Pu_initial./z_initial + 1;
i = 1;
cvx_solver SDPT3
while (1)
    
    uav_destination = (x_cu - x_initial).^2 + (y_cu - y_initial).^2 + h^2;
    uav_eavesdropper = (x_e - x_initial).^2 + (y_e - y_initial).^2 + h^2;
    uav_primary = (x_pu - x_initial).^2 + (y_pu - y_initial).^2 + h^2;
    A = exp(-index)*gamma*d_cs_cu./(gamma*Pu_initial./uav_destination + 1);
    B = gamma*d_cs_e./(gamma*Pu_initial./uav_eavesdropper + 1);
    
    cvx_begin
        variables Pcs_cvx(1,N) 
        variables d_cvx(1,N) t_0_cvx(1,N) e_cvx(1,N) k_cvx(1,N);    
        maximize(sum(d_cvx - e_cvx)/N) % (log2(1 + B.*Pcs_initial) + B.*(Pcs_cvx - Pcs_initial)./((1 + B.*Pcs_initial)*log(2)))
        subject to
            %%% - Power constraints - %%%
            (1/N)*sum(Pcs_cvx(1,1:N)) <= Pcs_average;
            
            %%% - Cognitive radio constraint - %%%
            (1/N)*sum(beta*d_cs_pu*Pcs_cvx(1,1:N) + (beta./uav_primary(1,1:N)).*Pu_initial(1,1:N)) <= 10^(-5);
            
            for n = 1:1:N
                %%% - Power constraints - %%%
                Pcs_cvx(n) >= 0;
                Pcs_cvx(n) <= Pcs_max;
                
                %%% - Rd constraints - %%%
                d_cvx(n) <= log2(1 + 1/t_0_initial(n)) - (t_0_cvx(n) - t_0_initial(n))/(t_0_initial(n)*(t_0_initial(n) + 1)*log(2));
                norm([1 (Pcs_cvx(n) - t_0_cvx(n))*0.5*sqrt(A(n))]) <= (Pcs_cvx(n) + t_0_cvx(n))*0.5*sqrt(A(n));
                
                %%% - Re constraints - %%%
                e_cvx(n) >= log2(1 + B(n)*Pcs_initial(n)) + B(n)*(Pcs_cvx(n) - Pcs_initial(n))/((1 + B(n)*Pcs_initial(n))*log(2));
                
                %%% - Implicit constraints - %%%
                Pcs_cvx(n) + t_0_cvx(n) >= 0;
                t_0_cvx(n) >= 0;   
            end
    cvx_end
    for n = 1:1:length(A)
        if (A(n) > B(n))
            Pcs_initial(n) = Pcs_cvx(n);   
        else
            Pcs_initial(n) = 0;
        end
    end
    %Pcs_initial = Pcs_cvx;
    t_0_initial = t_0_cvx;
    
    uav_destination = (x_cu - x_initial).^2 + (y_cu - y_initial).^2 + h^2;
    uav_eavesdropper = (x_e - x_initial).^2 + (y_e - y_initial).^2 + h^2;
    uav_primary = (x_pu - x_initial).^2 + (y_pu - y_initial).^2 + h^2;
    rate_d = log2(1 + exp(-index)*gamma*d_cs_cu*Pcs_initial./(gamma*Pu_initial./uav_destination + 1));
    rate_e = log2(1 + gamma*d_cs_e*Pcs_initial./(gamma*Pu_initial./uav_eavesdropper + 1));
    rate_s = rate_d - rate_e;
    rate_average = (1/N)*sum(rate_s(1,1:N));
    disp(rate_average);
    
    C = exp(-index)*gamma*d_cs_cu.*Pcs_initial;
    D = gamma./uav_destination;
    E = gamma.*d_cs_e.*Pcs_initial;
    F = gamma./uav_eavesdropper;
    G = (-C.*D)./((D.*Pu_initial + 1).*(D.*Pu_initial + C + 1)*log(2));
    H = log2(1 + C./(D.*Pu_initial + 1));
    
    cvx_begin
        variables Pu_cvx(1,N)
        variables e_cvx(1,N) m_cvx(1,N)
        maximize (sum(G.*(Pu_cvx - Pu_initial) + H - e_cvx)/N) %  
        subject to
            %%% - Power constraints - %%%
            (1/N)*sum(Pu_cvx(1,1:N)) <= Pu_average;
            
            %%% - Cognitive radio constraint - %%%
            (1/N)*sum(beta*d_cs_pu*Pcs_initial(1,1:N) + (beta./uav_primary(1,1:N)).*Pu_cvx(1,1:N)) <= 10^(-5);
            
            for n = 1:1:N
                %%% - Power constraints - %%%
                Pu_cvx(n) >= 0;
                Pu_cvx(n) <= Pu_max;
                
                %%% - Re constraints - %%%
                e_cvx(n) >= log2(1 + m_initial(n)) + (m_cvx(n) - m_initial(n))./((1 + m_initial(n))*log(2));
                norm([sqrt(E(n)) 0.5*(m_cvx(n) - F(n)*Pu_cvx(n) - 1)]) <= 0.5*(m_cvx(n) + F(n)*Pu_cvx(n) + 1);  
            end
    cvx_end
    Pu_initial = Pu_cvx;
    m_initial = m_cvx;
    
    uav_destination = (x_cu - x_initial).^2 + (y_cu - y_initial).^2 + h^2;
    uav_eavesdropper = (x_e - x_initial).^2 + (y_e - y_initial).^2 + h^2;
    uav_primary = (x_pu - x_initial).^2 + (y_pu - y_initial).^2 + h^2;
    rate_d = log2(1 + exp(-index)*gamma*d_cs_cu*Pcs_initial./(gamma*Pu_initial./uav_destination + 1));
    rate_e = log2(1 + gamma*d_cs_e*Pcs_initial./(gamma*Pu_initial./uav_eavesdropper + 1));
    rate_s = rate_d - rate_e;
    rate_average = (1/N)*sum(rate_s(1,1:N));
    disp(rate_average);
    
    if (i == 1)
        t_1_initial = gamma*Pu_initial./z_initial + 1;
    end
    
    C = exp(-index)*gamma*d_cs_cu.*Pcs_initial;
    E = gamma.*d_cs_e.*Pcs_initial;
    I = log2(1 + E.*k_initial./(k_initial + gamma*Pu_initial));
    J = (gamma*E.*Pu_initial)./(((E + 1).*k_initial + gamma*Pu_initial).*(k_initial + gamma*Pu_initial)*log(2));
    L = log2(1 + C./t_1_initial);
    M = -C./(t_1_initial.*(t_1_initial + C)*log(2));
    cvx_begin
        variables d_cvx(1,N) e_cvx(1,N) x_cvx(1,N) y_cvx(1,N) z_cvx(1,N) k_cvx(1,N) c_cvx(1,N) q_cvx(1,N) t_1_cvx(1,N) t_2_cvx(1,N);
        maximize (sum(d_cvx - e_cvx)/N)
        subject to
            
            %%% - Trajectory constraints - %%%
            norm([(x_cvx(1)-x0) (y_cvx(1)-y0)]) <= T_flight*V/N;
            for n = 1:1:N-1
                norm([(x_cvx(n+1)-x_cvx(n)) (y_cvx(n+1)-y_cvx(n))]) <= T_flight*V/N;
            end
            x_cvx(N) == xF;
            y_cvx(N) == yF;
            
            %%% - Cognitive radio constraint - %%%
            (1/N)*sum(beta*d_cs_pu*Pcs_initial(1,1:N)+c_cvx(1,1:N)) <= 10^(-5);
            
            for n = 1:1:N              
                norm([(x_e-x_cvx(n)) (y_e-y_cvx(n)) h (1/2)*(k_cvx(n)-1)]) <= (1/2)*(k_cvx(n)+1);
                e_cvx(n) >= I(n) + J(n)*(k_cvx(n) - k_initial(n));
                
                z_cvx(n) <= (x_cu-x_initial(n))^2+2*(x_initial(n)-x_cu)*(x_cvx(n)-x_initial(n))+(y_initial(n))^2+2*y_initial(n)*(y_cvx(n)-y_initial(n))+h^2;
                %norm([(gamma*Pu_initial(n) + z_cvx(n))*sqrt(K(n)) t_2_cvx(n)*sqrt(1/K(n)) 0.5*(t_1_cvx(n) - 1)]) <= 0.5*(t_1_cvx(n) + 1);
                gamma*Pu_initial(n)*t_2_cvx(n) + 1 <= t_1_cvx(n);
                norm([1 0.5*(t_2_cvx(n) - z_cvx(n))]) <= 0.5*(t_2_cvx(n) + z_cvx(n));
                L(n) + M(n)*(t_1_cvx(n) - t_1_initial(n)) >= d_cvx(n);
                
                %%% - Cognitive radio constraints - %%%
                q_cvx(n) <= (x_pu-x_initial(n))^2+2*(x_initial(n)-x_pu)*(x_cvx(n)-x_initial(n))+(y_pu-y_initial(n))^2+2*(y_initial(n)-y_pu)*(y_cvx(n)-y_initial(n)); 
                norm([sqrt(beta*Pu_initial(n)) 0.5*(c_cvx(n)-q_cvx(n))]) <= 0.5*(c_cvx(n)+q_cvx(n));
                
            end
    cvx_end
    x_inital = x_cvx;
    y_initial = y_cvx;
    k_initial = k_cvx;  
    z_initial = z_cvx;
    t_1_initial = t_1_cvx;
    
    uav_destination = (x_cu - x_initial).^2 + (y_cu - y_initial).^2 + h^2;
    uav_eavesdropper = (x_e - x_initial).^2 + (y_e - y_initial).^2 + h^2;
    uav_primary = (x_pu - x_initial).^2 + (y_pu - y_initial).^2 + h^2;
    rate_d = log2(1 + exp(-index)*gamma*d_cs_cu*Pcs_initial./(gamma*Pu_initial./uav_destination + 1));
    rate_e = log2(1 + gamma*d_cs_e*Pcs_initial./(gamma*Pu_initial./uav_eavesdropper + 1));
    rate_s = rate_d - rate_e;
    rate_average = (1/N)*sum(rate_s(1,1:N));
    disp(rate_average);
    i = i + 1;
    disp('Iteration');
    disp(i);
end































