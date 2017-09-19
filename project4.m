%Ranjeeth KS, University of Calgary

% here, Q and P matrix are set.. if only error states of azi and wz are
% corrected, it works fine.. 


clear all;
close all;
clc;
format long g;

gravity	 = 9.805209209982110;


we = 7.2921151467e-5; % rad/sec
a = 6378137; % semi major in m, axis WGS84
e2 = 0.00669437999014; % eccentricity square,  Department of Defense World Geodetic System 1984, Its Definition and Relationships with Local Geodetic Systems http://home.online.no/~sigurdhu/WGS84_Eng.html

a1=9.7803267714; 
a4=-0.0000030876910891;
a2=0.0052790414; 
a5=0.0000000043977311;
a3=0.0000232718; 
a6=0.0000000000007211;

g_p1 = 9.80520920998301; %9.80533396918467 ise gravity value at the first GPS solution
% Initial g
xbow_fx_bias =  4.6*1e-3*g_p1; %converted mg to g 
xbow_fx_sf = -0.02/100; %converted % to actual SF

xbow_fy_bias =  7.25*1e-3*g_p1; %converted mg to g 
xbow_fy_sf = -0.066/100; %converted % to actual SF

xbow_fz_bias =  -11.65*1e-3*g_p1; %converted mg to g 
xbow_fz_sf = -0.025/100; %converted % to actual SF

xbow_wx_bias = 0.079*pi/180; % No Conversion is needed, already in deg/ sec;
xbow_wx_sf = -0.05/100; %converted % to actual SF

xbow_wy_bias = 0.088*pi/180; % No Conversion is needed, already in deg/ sec;
xbow_wy_sf = 0.17/100; %converted % to actual SF 

xbow_wz_bias =0.03*pi/180; % 0.0198/ previously no bias/0.198/ 0.03
xbow_wz_sf = 0.17/100; %3.17/ previously no SF/0.17/ 0.17


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nova_fx_bias =  -0.48*1e-3*g_p1; %converted mg to m/s^2 
nova_fx_sf = 65/1000000; %converted ppm to actual SF


nova_fy_bias =  0.165*1e-3*g_p1; %converted mg to m/s^2 
nova_fy_sf = 45/1000000; %converted ppm to actual SF


nova_wz_bias = 2.5/(180*3600/pi); % converted deg/hr to rad/sec; 20.3/-2.3
nova_wz_sf = -15000/1000000; %converted ppm to actual SF; -1500/-20000

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

novatel_gps_pos = load('BESTGPSPOS.mat');
novatel_gps_vel = load('BESTGPSVEL.mat');
ins_PVA = load('INSPVAS.mat');
raw_nova_imu = load('RAWIMUS.mat');
raw_xbow_imu=load('Xbow_Apr_22_11.mat');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_t=1;
%car_vel_100hz = load('charchip_1Hz_intrap_vel.mat');
%car_time_100hz = load('charchip_1Hz_intrap_time.mat');

car_chip_1hz = load('CarChip_Speed_interpolated.mat');
car_vel_1hz_nosync = car_chip_1hz.CarChip_Speed_1HZ;
car_time_1hz = car_chip_1hz.CarChip_second_1HZ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bias_odo=mean(car_vel_1hz_nosync(1:length(ins_PVA.INS_vn))-(sqrt((ins_PVA.INS_vn).*(ins_PVA.INS_vn)+(ins_PVA.INS_ve).*(ins_PVA.INS_ve))));
car_vel_1hz_nosync=car_vel_1hz_nosync-(-0.2); %previously no bias
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


car_acc_1hz_nosync(1) = car_vel_1hz_nosync(1);
m=2:length(car_vel_1hz_nosync);
n=1:length(car_vel_1hz_nosync)-1; 
%o=2:length(car_vel_1hz);
acc_in = 2:length(car_vel_1hz_nosync);

car_acc_1hz_nosync(acc_in)=(car_vel_1hz_nosync(m)-car_vel_1hz_nosync(n))/delta_t;
%car_at_1hz(acc_in)=(car_time_1hz(m)-car_time_1hz(n))/delta_t;

nova_fx_raw = raw_nova_imu.f.x;
nova_fy_raw = -(raw_nova_imu.f.y);
nova_wz_raw = raw_nova_imu.w.z;



nova_fx_100Hz = (nova_fx_raw - nova_fx_bias)/(1+nova_fx_sf);  
nova_fy_100Hz = (nova_fy_raw - nova_fy_bias)/(1+nova_fy_sf);  
nova_wz_100Hz = (nova_wz_raw - nova_wz_bias)/(1+nova_wz_sf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xbow_fx_raw = raw_xbow_imu.f.x;
xbow_fy_raw = (raw_xbow_imu.f.y);
xbow_wz_raw = raw_xbow_imu.w.z;



xbow_fx_100Hz = (xbow_fx_raw - xbow_fx_bias)/(1+xbow_fx_sf);  
xbow_fy_100Hz = (xbow_fy_raw - xbow_fy_bias)/(1+xbow_fy_sf);  
xbow_wz_100Hz = (xbow_wz_raw - xbow_wz_bias)/(1+xbow_wz_sf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
start_from=1;
shift=13176 + 100*(start_from-1); % samples to sync GPS with IMU data
 x_shift = round(shift/100); % in seconds


nova_fx_100Hz = wden(nova_fx_100Hz, 'rigrsure', 's', 'one', 6, 'db5');
nova_fy_100Hz = wden(nova_fy_100Hz, 'rigrsure', 's', 'one', 6, 'db5');
nova_wz_100Hz = wden(nova_wz_100Hz, 'rigrsure', 's', 'one',6, 'db5');

% xbow_fx_100Hz = wden(xbow_fx_100Hz, 'rigrsure', 's', 'one', 6, 'db5');
% xbow_fy_100Hz = wden(xbow_fy_100Hz, 'rigrsure', 's', 'one', 6, 'db5');
% xbow_wz_100Hz = wden(xbow_wz_100Hz, 'rigrsure', 's', 'one',6, 'db5');


sync_from_xbow=1+shift;
ind=sync_from_xbow:100:length(nova_fx_100Hz);
nova_fx=nova_fx_100Hz(ind);

ind=sync_from_xbow:100:length(nova_fx_100Hz);
nova_fy=nova_fy_100Hz(ind);

ind=sync_from_xbow:100:length(nova_fx_100Hz);
nova_wz=nova_wz_100Hz(ind);

car_acc_1hz = car_acc_1hz_nosync((x_shift+1):length( car_acc_1hz_nosync));
car_vel_1hz = car_vel_1hz_nosync((x_shift+1):length( car_vel_1hz_nosync));

corr_t_gyro = 3*3600;% seconds
std_gyro = .01*pi/180;% radians/sec

corr_t_aodo = 0.1*3600;% seconds
std_aodo       = 1; %meters/sec^2


extend_az = 0;
I_15X15 = eye(9);
I_3X3 = [1 0 0; 0 1 0; 0 0 1];
O_3X3= [0 0 0; 0 0 0; 0 0 0];

q_scale=0.00000000000000000000000000000000001;
p_scale=2;
R_Scale=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        phi(1) = ins_PVA.INS_Lat(1+x_shift)*pi/180; % at time 490977.001
        lambda(1) = ins_PVA.INS_Long(1+x_shift)*pi/180; % at time 490977.001
        height(1) = ins_PVA.INS_Alt(1+x_shift); % at time 490977.001
        v_e(1)=ins_PVA.INS_ve(1+x_shift);
        v_n(1)=ins_PVA.INS_vn(1+x_shift);
        v_u(1)=ins_PVA.INS_vu(1+x_shift);
        Azi(1)=ins_PVA.INS_Azi(1+x_shift);
        az = Azi(1)*pi/180;
        
        G_matrix=[1 0 0 0 0 0 0 0 0;            %do not change this
          0 1 0 0 0 0 0 0 0; 
          0 0 1 0 0 0 0 0 0; 
          0 0 0 1 0 0 0 0 0;
          0 0 0 0 1 0 0 0 0;
          0 0 0 0 0 1 0 0 0;
          0 0 0 0 0 0 1 0 0;
          0 0 0 0 0 0 0 sqrt(2/corr_t_aodo)*delta_t 0;
          0 0 0 0 0 0 0 0 sqrt(2/corr_t_gyro)*delta_t];

        H_matrix =[1 0 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0;
        0 0 1 0 0 0 0 0 0;                   %do not change this
        0 0 0 1 0 0 0 0 0;
        0 0 0 0 1 0 0 0 0;
        0 0 0 0 0 1 0 0 0];
    
    
       
        N_vector = 1*[  9.39341994706799e-06*pi/180;         %computed from GPS trajectory and true trajector
                     6.45014816832279e-06*pi/180; 
                    0.743402554849468;
                    0.0437468935289519;
                    0.080828777420849;
                    0.0741750197257357];

        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

            g = a1*(1+a2*sin(phi(1))*sin(phi(1))+a3*sin(phi(1))*sin(phi(1))*sin(phi(1))*sin(phi(1)))+(a4+a5*sin(phi(1))*sin(phi(1)))*height(1)+a6*height(1)*height(1);  % at time 490977.001
            R_N = a./sqrt(1-e2*sin(phi(1)).*sin(phi(1)));% Normal radius
            R_M = (a*(1-e2))./((1-e2*sin(phi(1)).*sin(phi(1))).^(1.5));% Meridian radius

            p=(asin((nova_fy(1)-car_acc_1hz(1))/g));
            pitch(1)=p*180/pi;

            r=-(asin((nova_fx(1)+(car_vel_1hz(1)*nova_wz(1)))/(g*cos(p))));
            roll(1)=r*180/pi;


            F1 = [0 (1*delta_t/(R_M + height(1))) 0; (1*delta_t/(cos(phi(1))*(R_N + height(1)))) 0 0; 0 0 delta_t];
            F2 = [car_acc_1hz(1)*cos(az)*cos(p)*delta_t sin(az)*cos(p)*delta_t 0; -car_acc_1hz(1)*sin(az)*cos(p)*delta_t cos(az)*cos(p)*delta_t 0; 0 sin(p) 0];
            F3= [1 0 -delta_t; 0 (1-delta_t/corr_t_aodo) 0; 0 0 (1-delta_t/corr_t_gyro)] ;
            Phi_matrix  = [I_3X3 F1 O_3X3; 
                           O_3X3  I_3X3 F2; 
                           O_3X3  O_3X3 F3];

            if (extend_az == 1)
             F3= [1 0 -delta_t*cos(p)*cos(r); 0 (1-delta_t/corr_t_aodo) 0; 0 0 (1-delta_t/corr_t_gyro)] ;
            % F4= [delta_t*we*sin(phi(1)) 0 0; 0 0 0; 0 0 0];
             F4= [delta_t*we*sin(phi(1))+delta_t*v_e(1)*(R_N + height(1))/(((cos(phi(1)))*(R_N + height(1))).^2) 0 delta_t*(-v_e(1)*tan(phi(1))/((R_N + height(1)).^2)); 0 0 0; 0 0 0];
             F5 = [delta_t*((R_N + height(1)))*tan(phi(1))/((R_N + height(1)).^2) 0 0; 0 0 0; 0 0 0];

             Phi_matrix  = [I_3X3 F1 O_3X3; 
                            O_3X3  I_3X3 F2; 
                            F4  F5 F3];
            end
            
            
            x_post_vector = [10/(R_M + height(1));                       %computed from VERY SMAL values
                        10/((R_N + height(1))*cos(phi(1))); 
                        10; 
                        4; 
                        4; 
                        4; 
                        1*pi/180;
                        1; 
                        (0.005*pi/180)/delta_t]; %0.005
   
     
              W_vector = 0.5*[2/(R_M + height(1));                %computed from VERY SMAL values
                        2/((R_N + height(1))*cos(phi(1))); 
                        2; 
                        2; 
                        2; 
                        2; 
                        1*pi/180;
                        std_aodo; 
                        std_gyro];

%             Q_matrix = zeros(9,9);
% 
%             Q_matrix(1,1)= (W_vector(1)).^2;
%             Q_matrix(2,2)= (W_vector(2)).^2;
%             Q_matrix(3,3)= (W_vector(3)).^2;
%             Q_matrix(4,4)= (W_vector(4)).^2;
%             Q_matrix(5,5)= (W_vector(5)).^2;
%             Q_matrix(6,6)= (W_vector(6)).^2;
%             Q_matrix(7,7)= (W_vector(7)).^2;
%             Q_matrix(8,8)= (W_vector(8)).^2;
%             Q_matrix(9,9)= (W_vector(9)).^2;

            Q_matrix = zeros(9,9);

            Q_matrix(1,1)= 0.001;
            Q_matrix(2,2)= 0.001;
            Q_matrix(3,3)= 2;
            Q_matrix(4,4)= 1;
            Q_matrix(5,5)= 1;
            Q_matrix(6,6)= 0.1;
            Q_matrix(7,7)= 0.005;
            Q_matrix(8,8)= 0.05;
            Q_matrix(9,9)= 0.001;
            Q_matrix=q_scale*Q_matrix;

            R_matrix = zeros(6,6);

            % 
            R_matrix(1,1)= (N_vector(1)).^2;
            R_matrix(2,2)= (N_vector(2)).^2;
            R_matrix(3,3)= (N_vector(3)).^2;
            R_matrix(4,4)= (N_vector(4)).^2;
            R_matrix(5,5)= (N_vector(5)).^2;
            R_matrix(6,6)= (N_vector(6)).^2;


            P_post_matrix = zeros(9,9);

%             P_post_matrix = Q_matrix;   % for simplicity  
%             Q_matrix = 4*Q_matrix;

            P_post_matrix(1,1)= (W_vector(1)).^2;
            P_post_matrix(2,2)= (W_vector(2)).^2;
            P_post_matrix(3,3)= (W_vector(3)).^2;
            P_post_matrix(4,4)= (W_vector(4)).^2;
            P_post_matrix(5,5)= (W_vector(5)).^2;
            P_post_matrix(6,6)= (W_vector(6)).^2;
            P_post_matrix(7,7)= 0.05; %radians
            P_post_matrix(8,8)= (W_vector(8)).^2;
            P_post_matrix(9,9)= (W_vector(9)).^2;

            P_post_matrix=p_scale*P_post_matrix;


            % 
            % 
            % D2R=pi/180;
            % R2D=180/pi;

            % lat_std_in_meters = novatel_gps_pos.GP_Lat_std(start_from); %k is the GPS epoch
            % long_std_in_meters = novatel_gps_pos.GP_Long_std(start_from); %k is the GPS epoch
            % N = a./sqrt(1-e2*sin(novatel_gps_pos.GP_Lat(start_from)*D2R).*sin(novatel_gps_pos.GP_Lat(start_from)*D2R));% D2R:deg to rad
            % M = a*(1-e2)./((1-e2*sin(novatel_gps_pos.GP_Lat(start_from)*D2R).*sin(novatel_gps_pos.GP_Lat(start_from)*D2R)).^(1.5));
            % lat_std_in_rad = lat_std_in_meters/(M+novatel_gps_pos.GP_Alt(start_from));
            % long_std_in_rad = long_std_in_meters/((N+novatel_gps_pos.GP_Alt(start_from))*cos(novatel_gps_pos.GP_Lat(start_from)*D2R));
            % R_matrix(1,1) = (lat_std_in_rad)^2;
            % R_matrix(2,2) = (long_std_in_rad)^2;
            % R_matrix(3,3) = (novatel_gps_pos.GP_Alt(start_from))^2;
            % R_matrix(4,4) = (long_std_in_meters*5)^2;
            % R_matrix(5,5) = (lat_std_in_meters*5)^2;
            % R_matrix(6,6) = (novatel_gps_pos.GP_Alt(start_from)*10)^2;
            % R_matrix = R_matrix* R_Scale;

            GPS_update = 1; % always GPS is available
            close_loop =1;

        
                plot_len=1700;
                 for te=2:plot_len;
%                      if(te>1)
%                      nova_wz(te)=nova_wz(te)-x_post_vector(9);
%                      x_post_vector(7)=0;
%                      end
                     p=(asin((nova_fy(te)-car_acc_1hz(te))/g)); % prev g, because prev phi, h is known
                     pitch(te)=p*180/pi;

                     r=-(asin((nova_fx(te)+(car_vel_1hz(te)*nova_wz(te)))/(g*cos(p)))); % prev g, because prev phi, h is known
                     roll(te)=r*180/pi;

                     az = -(nova_wz(te)*(cos(p)*cos(r)) - we*sin(phi(te-1)) - ((v_e(te-1)*tan(phi(te-1)))/(R_N + height(te-1))))*delta_t + Azi(te-1)*pi/180; % first term present, next terms previous

                     Azi(te)=  az*180/pi;


                     v_e(te)= car_vel_1hz(te)*sin(az)*cos(p);
                     v_n(te)= car_vel_1hz(te)*cos(az)*cos(p);
                     v_u(te)= car_vel_1hz(te)*sin(p);

                     height(te)=height(te-1)+v_u(te)*delta_t;
                     phi(te)=phi(te-1)+ v_n(te)*delta_t/(R_M + height(te));
                     lambda(te)=lambda(te-1)+ v_e(te)*delta_t/((R_N + height(te))*cos(phi(te)));

                     g = a1*(1+a2*sin(phi(te))*sin(phi(te))+a3*sin(phi(te))*sin(phi(te))*sin(phi(te))*sin(phi(te)))+(a4+a5*sin(phi(1))*sin(phi(1)))*height(1)+a6*height(1)*height(1);  % at time 490977.001
                     R_N = a./sqrt(1-e2*sin(phi(te)).*sin(phi(te)));% Normal radius
                     R_M = (a*(1-e2))./((1-e2*sin(phi(te)).*sin(phi(te))).^(1.5));% Meridian radius

                     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER

                    %%%% PHI matrix computed for present epoch 
                    F1 = [0 (1*delta_t/(R_M + height(te))) 0; (1*delta_t/(cos(phi(te))*(R_N + height(te)))) 0 0; 0 0 delta_t]; 
                    F2 = [car_acc_1hz(te)*cos(az)*cos(p)*delta_t sin(az)*cos(p)*delta_t 0; -car_acc_1hz(te)*sin(az)*cos(p)*delta_t cos(az)*cos(p)*delta_t 0; 0 sin(p) 0];
                    F3= [1 0 -delta_t; 0 (1-delta_t/corr_t_aodo) 0; 0 0 (1-delta_t/corr_t_gyro)] ;

                    %%%% PHI matrix computed for previous epoch
                %     F1 = [0 (1*delta_t/(R_M + height(te-1))) 0; (1*delta_t/(cos(phi(te-1))*(R_N + height(te-1)))) 0 0; 0 0 delta_t]; 
                %     F2 = [car_acc_1hz(te-1)*cos((Azi(te-1))*pi/180)*cos((pitch(te-1))*pi/180)*delta_t sin((Azi(te-1))*pi/180)*cos((pitch(te-1))*pi/180)*delta_t 0; -car_acc_1hz(te-1)*sin((Azi(te-1))*pi/180)*cos((pitch(te-1))*pi/180)*delta_t cos((Azi(te-1))*pi/180)*cos((pitch(te-1))*pi/180)*delta_t 0; 0 sin((pitch(te-1))*pi/180) 0];
                %     F3= [1 0 -delta_t; 0 (1-delta_t/corr_t_aodo) 0; 0 0 (1-delta_t/corr_t_gyro)] ;

                    Phi_matrix  = [I_3X3 F1 O_3X3; 
                                   O_3X3  I_3X3 F2; 
                                   O_3X3  O_3X3 F3];               % Phi matrix computation

                if (extend_az == 1)
                     F3= [1 0 -delta_t*cos(p)*cos(r); 0 (1-delta_t/corr_t_aodo) 0; 0 0 (1-delta_t/corr_t_gyro)] ;
                %     F4= [delta_t*we*sin(phi(te)) 0 0; 0 0 0; 0 0 0];
                    F4= [delta_t*we*sin(phi(te))+delta_t*v_e(te)*(R_N + height(te))/(((cos(phi(te)))*(R_N + height(te))).^2) 0 delta_t*(-v_e(te)*tan(phi(te))/((R_N + height(te)).^2)); 0 0 0; 0 0 0];
                     F5 = [delta_t*((R_N + height(te)))*tan(phi(te))/((R_N + height(te)).^2) 0 0; 0 0 0; 0 0 0];

                    Phi_matrix  = [I_3X3 F1 O_3X3; 
                                   O_3X3  I_3X3 F2; 
                                   F4  F5 F3];
                end

                    x_pre_vector=Phi_matrix*x_post_vector;

                    P_pre_matrix = Phi_matrix*P_post_matrix*(Phi_matrix)' + G_matrix*Q_matrix*G_matrix';

                    P_pre_matrix =( P_pre_matrix + P_pre_matrix')/2;

                    if GPS_update == 1

                %         lat_std_in_meters = novatel_gps_pos.GP_Lat_std(te+start_from-1); %k is the GPS epoch
                %         long_std_in_meters = novatel_gps_pos.GP_Long_std(te+start_from-1); %k is the GPS epoch
                %         N = a./sqrt(1-e2*sin(novatel_gps_pos.GP_Lat(te+start_from-1)*D2R).*sin(novatel_gps_pos.GP_Lat(te+start_from-1)*D2R));% D2R:deg to rad
                %         M = a*(1-e2)./((1-e2*sin(novatel_gps_pos.GP_Lat(te+start_from-1)*D2R).*sin(novatel_gps_pos.GP_Lat(te+start_from-1)*D2R)).^(1.5));
                %         lat_std_in_rad = lat_std_in_meters/(M+novatel_gps_pos.GP_Alt(te+start_from-1));
                %         long_std_in_rad = long_std_in_meters/((N+novatel_gps_pos.GP_Alt(te+start_from-1))*cos(novatel_gps_pos.GP_Lat(te+start_from-1)*D2R));
                %         R_matrix(1,1) = (lat_std_in_rad)^2;
                %         R_matrix(2,2) = (long_std_in_rad)^2;
                %         R_matrix(3,3) = (novatel_gps_pos.GP_Alt(te+start_from-1))^2;
                %         R_matrix(4,4) = (long_std_in_meters*5)^2;
                %         R_matrix(5,5) = (lat_std_in_meters*5)^2;
                %         R_matrix(6,6) = (novatel_gps_pos.GP_Alt(te+start_from-1)*10)^2;





                    Z_vector=[phi(te)-ins_PVA.INS_Lat(te+x_shift)*pi/180;
                            lambda(te)-ins_PVA.INS_Long(te+x_shift)*pi/180;
                            height(te)-ins_PVA.INS_Alt(te+x_shift); 
                            v_e(te)-ins_PVA.INS_ve(te+x_shift);
                            v_n(te)-ins_PVA.INS_vn(te+x_shift);
                            v_u(te)-ins_PVA.INS_vu(te+x_shift)];
                    K_matrix = P_pre_matrix*(H_matrix)'*(inv(((H_matrix*P_pre_matrix*(H_matrix)')+(R_matrix))));
                    x_post_vector = x_pre_vector + K_matrix*(Z_vector - H_matrix*x_pre_vector );
                    P_post_matrix = (I_15X15-K_matrix*H_matrix)*P_pre_matrix;
                    %P_post_matrix = (I_15X15-K_matrix*H_matrix)*P_pre_matrix*(I_15X15-K_matrix*H_matrix)' + K_matrix*R_matrix*K_matrix'; % Joseph form

                    else
                    x_post_vector = x_pre_vector ;
                    P_post_matrix=P_pre_matrix    ;
                    end

                    P_post_matrix = (P_post_matrix+P_post_matrix')/2;




                    KF_n_phi(te) = (phi(te)- x_post_vector(1))*180/pi;
                    KF_n_lambda(te) = (lambda(te)- x_post_vector(2))*180/pi;
                    KF_n_alt(te) = height(te)- x_post_vector(3);
                    KF_n_ve(te) = v_e(te)- x_post_vector(4);
                    KF_n_vn(te) = v_n(te)- x_post_vector(5);
                    KF_n_vu(te) = v_u(te)- x_post_vector(6);
                    KF_n_Azi(te) = (az- x_post_vector(7))*180/pi;

                err_ol_phi(te-1)=x_post_vector(1);
                err_ol_lambda(te-1)=x_post_vector(2);
                err_ol_alt(te-1)=x_post_vector(3);
                err_ol_ve(te-1)=x_post_vector(4);
                err_ol_vn(te-1)=x_post_vector(5);
                err_ol_vu(te-1)=x_post_vector(6);
                err_ol_azi(te-1)=x_post_vector(7);
                err_ol_aodo(te-1)=x_post_vector(8);
                err_ol_wz(te-1)=x_post_vector(9);





                 end
                
                 
                n_ol_phi=phi*180/pi;
                n_ol_lambda=lambda*180/pi;
                n_ol_height=height;
                n_ol_ve = v_e;
                n_ol_vn = v_n;
                n_ol_vu = v_u;
                n_ol_pitch = pitch;
                n_ol_roll = roll;
                n_ol_Azi = Azi;

                
                KF_n_ol_phi=KF_n_phi;
                KF_n_ol_lambda=KF_n_lambda;
                KF_n_ol_alt=KF_n_alt;
                KF_n_ol_ve=KF_n_ve;
                KF_n_ol_vn=KF_n_vn;
                KF_n_ol_vu=KF_n_vu;
                KF_n_ol_Azi=KF_n_Azi;
                
                KF_n_ol_phi(1) = phi(1)*180/pi;
                KF_n_ol_lambda(1) = lambda(1)*180/pi;
                KF_n_ol_alt(1) = height(1);
                KF_n_ol_ve(1) = v_e(1);
                KF_n_ol_vn(1) = v_n(1);
                KF_n_ol_vu(1) = v_u(1);
                KF_n_ol_Azi(1) = Azi(1);


  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
  
   phi(1) = ins_PVA.INS_Lat(1+x_shift)*pi/180; % at time 490977.001
        lambda(1) = ins_PVA.INS_Long(1+x_shift)*pi/180; % at time 490977.001
        height(1) = ins_PVA.INS_Alt(1+x_shift); % at time 490977.001
        v_e(1)=ins_PVA.INS_ve(1+x_shift);
        v_n(1)=ins_PVA.INS_vn(1+x_shift);
        v_u(1)=ins_PVA.INS_vu(1+x_shift);
        Azi(1)=ins_PVA.INS_Azi(1+x_shift);
        az = Azi(1)*pi/180;
        
        G_matrix=[1 0 0 0 0 0 0 0 0;            %do not change this
          0 1 0 0 0 0 0 0 0; 
          0 0 1 0 0 0 0 0 0; 
          0 0 0 1 0 0 0 0 0;
          0 0 0 0 1 0 0 0 0;
          0 0 0 0 0 1 0 0 0;
          0 0 0 0 0 0 1 0 0;
          0 0 0 0 0 0 0 sqrt(2/corr_t_aodo)*delta_t 0;
          0 0 0 0 0 0 0 0 sqrt(2/corr_t_gyro)*delta_t];

        H_matrix =[1 0 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0;
        0 0 1 0 0 0 0 0 0;                   %do not change this
        0 0 0 1 0 0 0 0 0;
        0 0 0 0 1 0 0 0 0;
        0 0 0 0 0 1 0 0 0];
    
    
       
        N_vector = 1*[  9.39341994706799e-06*pi/180;         %computed from GPS trajectory and true trajector
                     6.45014816832279e-06*pi/180; 
                    0.743402554849468;
                    0.0437468935289519;
                    0.080828777420849;
                    0.0741750197257357];
                
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
            g = a1*(1+a2*sin(phi(1))*sin(phi(1))+a3*sin(phi(1))*sin(phi(1))*sin(phi(1))*sin(phi(1)))+(a4+a5*sin(phi(1))*sin(phi(1)))*height(1)+a6*height(1)*height(1);  % at time 490977.001
            R_N = a./sqrt(1-e2*sin(phi(1)).*sin(phi(1)));% Normal radius
            R_M = (a*(1-e2))./((1-e2*sin(phi(1)).*sin(phi(1))).^(1.5));% Meridian radius

            p=(asin((nova_fy(1)-car_acc_1hz(1))/g));
            pitch(1)=p*180/pi;

            r=-(asin((nova_fx(1)+(car_vel_1hz(1)*nova_wz(1)))/(g*cos(p))));
            roll(1)=r*180/pi;


            F1 = [0 (1*delta_t/(R_M + height(1))) 0; (1*delta_t/(cos(phi(1))*(R_N + height(1)))) 0 0; 0 0 delta_t];
            F2 = [car_acc_1hz(1)*cos(az)*cos(p)*delta_t sin(az)*cos(p)*delta_t 0; -car_acc_1hz(1)*sin(az)*cos(p)*delta_t cos(az)*cos(p)*delta_t 0; 0 sin(p) 0];
            F3= [1 0 -delta_t; 0 (1-delta_t/corr_t_aodo) 0; 0 0 (1-delta_t/corr_t_gyro)] ;
            Phi_matrix  = [I_3X3 F1 O_3X3; 
                           O_3X3  I_3X3 F2; 
                           O_3X3  O_3X3 F3];

            if (extend_az == 1)
             F3= [1 0 -delta_t*cos(p)*cos(r); 0 (1-delta_t/corr_t_aodo) 0; 0 0 (1-delta_t/corr_t_gyro)] ;
            % F4= [delta_t*we*sin(phi(1)) 0 0; 0 0 0; 0 0 0];
             F4= [delta_t*we*sin(phi(1))+delta_t*v_e(1)*(R_N + height(1))/(((cos(phi(1)))*(R_N + height(1))).^2) 0 delta_t*(-v_e(1)*tan(phi(1))/((R_N + height(1)).^2)); 0 0 0; 0 0 0];
             F5 = [delta_t*((R_N + height(1)))*tan(phi(1))/((R_N + height(1)).^2) 0 0; 0 0 0; 0 0 0];

             Phi_matrix  = [I_3X3 F1 O_3X3; 
                            O_3X3  I_3X3 F2; 
                            F4  F5 F3];
            end

            x_post_vector = [10/(R_M + height(1));                       %computed from VERY SMAL values
                        10/((R_N + height(1))*cos(phi(1))); 
                        10; 
                        4; 
                        4; 
                        4; 
                        1*pi/180;
                        1; 
                        (0.005*pi/180)/delta_t]; %0.005
   
     
              W_vector = 0.5*[2/(R_M + height(1));                %computed from VERY SMAL values
                        2/((R_N + height(1))*cos(phi(1))); 
                        2; 
                        2; 
                        2; 
                        2; 
                        1*pi/180;
                        std_aodo; 
                        std_gyro];

            Q_matrix = zeros(9,9);

            Q_matrix(1,1)= 0.001;
            Q_matrix(2,2)= 0.001;
            Q_matrix(3,3)= 2;
            Q_matrix(4,4)= 1;
            Q_matrix(5,5)= 1;
            Q_matrix(6,6)= 0.1;
            Q_matrix(7,7)= 0.001;
            Q_matrix(8,8)= 0.5;
            Q_matrix(9,9)= 0.001;

            Q_matrix=q_scale*Q_matrix;
            
            R_matrix = zeros(6,6);

            % 
            R_matrix(1,1)= (N_vector(1)).^2;
            R_matrix(2,2)= (N_vector(2)).^2;
            R_matrix(3,3)= (N_vector(3)).^2;
            R_matrix(4,4)= (N_vector(4)).^2;
            R_matrix(5,5)= (N_vector(5)).^2;
            R_matrix(6,6)= (N_vector(6)).^2;


            P_post_matrix = zeros(9,9);

%             P_post_matrix = Q_matrix;   % for simplicity  
%             Q_matrix = 4*Q_matrix;

            P_post_matrix(1,1)= (W_vector(1)).^2;
            P_post_matrix(2,2)= (W_vector(2)).^2;
            P_post_matrix(3,3)= (W_vector(3)).^2;
            P_post_matrix(4,4)= (W_vector(4)).^2;
            P_post_matrix(5,5)= (W_vector(5)).^2;
            P_post_matrix(6,6)= (W_vector(6)).^2;
            P_post_matrix(7,7)= 0.05; %radians
            P_post_matrix(8,8)= (W_vector(8)).^2;
            P_post_matrix(9,9)= (W_vector(9)).^2;

            P_post_matrix=p_scale*P_post_matrix;


           
%             
%             D2R=pi/180;
%             R2D=180/pi;
% 
%             lat_std_in_meters = novatel_gps_pos.GP_Lat_std(start_from); %k is the GPS epoch
%             long_std_in_meters = novatel_gps_pos.GP_Long_std(start_from); %k is the GPS epoch
%             N = a./sqrt(1-e2*sin(novatel_gps_pos.GP_Lat(start_from)*D2R).*sin(novatel_gps_pos.GP_Lat(start_from)*D2R));% D2R:deg to rad
%             M = a*(1-e2)./((1-e2*sin(novatel_gps_pos.GP_Lat(start_from)*D2R).*sin(novatel_gps_pos.GP_Lat(start_from)*D2R)).^(1.5));
%             lat_std_in_rad = lat_std_in_meters/(M+novatel_gps_pos.GP_Alt(start_from));
%             long_std_in_rad = long_std_in_meters/((N+novatel_gps_pos.GP_Alt(start_from))*cos(novatel_gps_pos.GP_Lat(start_from)*D2R));
%             R_matrix(1,1) = (lat_std_in_rad)^2;
%             R_matrix(2,2) = (long_std_in_rad)^2;
%             R_matrix(3,3) = (novatel_gps_pos.GP_Alt(start_from))^2;
%             R_matrix(4,4) = (long_std_in_meters*5)^2;
%             R_matrix(5,5) = (lat_std_in_meters*5)^2;
%             R_matrix(6,6) = (novatel_gps_pos.GP_Alt(start_from)*10)^2;
%             R_matrix = R_matrix* R_Scale;

            GPS_update = 1; % always GPS is available
            close_loop =1;
            extend_az=1;
            d_vodo_k_1 = 0;
                plot_len=1700;
                 for te=2:plot_len;
                     if(te>1)
                     nova_wz(te)=nova_wz(te)-x_post_vector(9);
                     x_post_vector(9)=0;
                      x_post_vector(7)=0;
%                       x_post_vector(1)=0;
%                       x_post_vector(2)=0;
%                       x_post_vector(3)=0;
%                       x_post_vector(4)=0;
%                       x_post_vector(5)=0;
%                        x_post_vector(6)=0;
%                      x_post_vector(8)=0;
                     end
                     p=(asin((nova_fy(te)-car_acc_1hz(te))/g)); % prev g, because prev phi, h is known
                     pitch(te)=p*180/pi;

                     r=-(asin((nova_fx(te)+(car_vel_1hz(te)*nova_wz(te)))/(g*cos(p)))); % prev g, because prev phi, h is known
                     roll(te)=r*180/pi;

                     az = -(nova_wz(te)*(cos(p)*cos(r)) - we*sin(phi(te-1)) - ((v_e(te-1)*tan(phi(te-1)))/(R_N + height(te-1))))*delta_t + Azi(te-1)*pi/180; % first term present, next terms previous

                     Azi(te)=  az*180/pi;


                     v_e(te)= car_vel_1hz(te)*sin(az)*cos(p);
                     v_n(te)= car_vel_1hz(te)*cos(az)*cos(p);
                     v_u(te)= car_vel_1hz(te)*sin(p);

                     height(te)=height(te-1)+v_u(te)*delta_t;
                     phi(te)=phi(te-1)+ v_n(te)*delta_t/(R_M + height(te));
                     lambda(te)=lambda(te-1)+ v_e(te)*delta_t/((R_N + height(te))*cos(phi(te)));

                     g = a1*(1+a2*sin(phi(te))*sin(phi(te))+a3*sin(phi(te))*sin(phi(te))*sin(phi(te))*sin(phi(te)))+(a4+a5*sin(phi(1))*sin(phi(1)))*height(1)+a6*height(1)*height(1);  % at time 490977.001
                     R_N = a./sqrt(1-e2*sin(phi(te)).*sin(phi(te)));% Normal radius
                     R_M = (a*(1-e2))./((1-e2*sin(phi(te)).*sin(phi(te))).^(1.5));% Meridian radius

                     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN FILTER

                    %%%% PHI matrix computed for present epoch 
                    F1 = [0 (1*delta_t/(R_M + height(te))) 0; (1*delta_t/(cos(phi(te))*(R_N + height(te)))) 0 0; 0 0 delta_t]; 
                    F2 = [car_acc_1hz(te)*cos(az)*cos(p)*delta_t sin(az)*cos(p)*delta_t 0; -car_acc_1hz(te)*sin(az)*cos(p)*delta_t cos(az)*cos(p)*delta_t 0; 0 sin(p) 0];
                    F3= [1 0 -delta_t; 0 (1-delta_t/corr_t_aodo) 0; 0 0 (1-delta_t/corr_t_gyro)] ;

                    %%%% PHI matrix computed for previous epoch
                %     F1 = [0 (1*delta_t/(R_M + height(te-1))) 0; (1*delta_t/(cos(phi(te-1))*(R_N + height(te-1)))) 0 0; 0 0 delta_t]; 
                %     F2 = [car_acc_1hz(te-1)*cos((Azi(te-1))*pi/180)*cos((pitch(te-1))*pi/180)*delta_t sin((Azi(te-1))*pi/180)*cos((pitch(te-1))*pi/180)*delta_t 0; -car_acc_1hz(te-1)*sin((Azi(te-1))*pi/180)*cos((pitch(te-1))*pi/180)*delta_t cos((Azi(te-1))*pi/180)*cos((pitch(te-1))*pi/180)*delta_t 0; 0 sin((pitch(te-1))*pi/180) 0];
                %     F3= [1 0 -delta_t; 0 (1-delta_t/corr_t_aodo) 0; 0 0 (1-delta_t/corr_t_gyro)] ;

                    Phi_matrix  = [I_3X3 F1 O_3X3; 
                                   O_3X3  I_3X3 F2; 
                                   O_3X3  O_3X3 F3];               % Phi matrix computation

                if (extend_az == 1)
                     F3= [1 0 -delta_t*cos(p)*cos(r); 0 (1-delta_t/corr_t_aodo) 0; 0 0 (1-delta_t/corr_t_gyro)] ;
                     %F4= [delta_t*we*sin(phi(te)) 0 0; 0 0 0; 0 0 0];
                    F4= [delta_t*we*sin(phi(te))+delta_t*v_e(te)*(R_N + height(te))/(((cos(phi(te)))*(R_N + height(te))).^2) 0 delta_t*(-v_e(te)*tan(phi(te))/((R_N + height(te)).^2)); 0 0 0; 0 0 0];
                     F5 = [delta_t*((R_N + height(te)))*tan(phi(te))/((R_N + height(te)).^2) 0 0; 0 0 0; 0 0 0];

                    Phi_matrix  = [I_3X3 F1 O_3X3; 
                                   O_3X3  I_3X3 F2; 
                                   F4  F5 F3];
                end

                    x_pre_vector=Phi_matrix*x_post_vector;

                    P_pre_matrix = Phi_matrix*P_post_matrix*(Phi_matrix)' + G_matrix*Q_matrix*G_matrix';
                    
                    P_pre_matrix =( P_pre_matrix + P_pre_matrix')/2;
%                     dummy_matrix =( P_pre_matrix + P_pre_matrix')/2;
%                     P_pre_matrix=eye(9);
%                     
%                     P_pre_matrix(1,1) = dummy_matrix(1,1);
%                     P_pre_matrix(2,2) = dummy_matrix(2,2);
%                     P_pre_matrix(3,3) = dummy_matrix(3,3);
%                     P_pre_matrix(4,4) = dummy_matrix(4,4);
%                     P_pre_matrix(5,5) = dummy_matrix(5,5);
%                     P_pre_matrix(6,6) = dummy_matrix(6,6);
%                     P_pre_matrix(7,7) = dummy_matrix(7,7);
%                     P_pre_matrix(8,8) = dummy_matrix(8,8);
%                     P_pre_matrix(9,9) = dummy_matrix(9,9);

                    if GPS_update == 1

%                         lat_std_in_meters = novatel_gps_pos.GP_Lat_std(te+start_from-1); %k is the GPS epoch
%                         long_std_in_meters = novatel_gps_pos.GP_Long_std(te+start_from-1); %k is the GPS epoch
%                         N = a./sqrt(1-e2*sin(novatel_gps_pos.GP_Lat(te+start_from-1)*D2R).*sin(novatel_gps_pos.GP_Lat(te+start_from-1)*D2R));% D2R:deg to rad
%                         M = a*(1-e2)./((1-e2*sin(novatel_gps_pos.GP_Lat(te+start_from-1)*D2R).*sin(novatel_gps_pos.GP_Lat(te+start_from-1)*D2R)).^(1.5));
%                         lat_std_in_rad = lat_std_in_meters/(M+novatel_gps_pos.GP_Alt(te+start_from-1));
%                         long_std_in_rad = long_std_in_meters/((N+novatel_gps_pos.GP_Alt(te+start_from-1))*cos(novatel_gps_pos.GP_Lat(te+start_from-1)*D2R));
%                         R_matrix(1,1) = (lat_std_in_rad)^2;
%                         R_matrix(2,2) = (long_std_in_rad)^2;
%                         R_matrix(3,3) = (novatel_gps_pos.GP_Alt(te+start_from-1))^2;
%                         R_matrix(4,4) = (long_std_in_meters*5)^2;
%                         R_matrix(5,5) = (lat_std_in_meters*5)^2;
%                         R_matrix(6,6) = (novatel_gps_pos.GP_Alt(te+start_from-1)*10)^2;
%                         %R_matrix=R_matrix*scale




                    Z_vector=[phi(te)-ins_PVA.INS_Lat(te+x_shift)*pi/180;
                            lambda(te)-ins_PVA.INS_Long(te+x_shift)*pi/180;
                            height(te)-ins_PVA.INS_Alt(te+x_shift); 
                            v_e(te)-ins_PVA.INS_ve(te+x_shift);
                            v_n(te)-ins_PVA.INS_vn(te+x_shift);
                            v_u(te)-ins_PVA.INS_vu(te+x_shift)];
                    K_matrix = P_pre_matrix*(H_matrix)'*(inv(((H_matrix*P_pre_matrix*(H_matrix)')+(R_matrix))));
                    x_post_vector = x_pre_vector + K_matrix*(Z_vector - H_matrix*x_pre_vector );
                    P_post_matrix = (I_15X15-K_matrix*H_matrix)*P_pre_matrix;
                    %P_post_matrix = (I_15X15-K_matrix*H_matrix)*P_pre_matrix*(I_15X15-K_matrix*H_matrix)' + K_matrix*R_matrix*K_matrix'; % Joseph form

                    else
                    x_post_vector = x_pre_vector ;
                    P_post_matrix=P_pre_matrix    ;
                    end
                    
                    P_post_matrix = (P_post_matrix+P_post_matrix')/2;
%                     dummy_matrix = (P_post_matrix+P_post_matrix')/2;
%                     P_post_matrix=eye(9);
%                     P_post_matrix(1,1) = dummy_matrix(1,1);
%                     P_post_matrix(2,2) = dummy_matrix(2,2);
%                     P_post_matrix(3,3) = dummy_matrix(3,3);
%                     P_post_matrix(4,4) = dummy_matrix(4,4);
%                     P_post_matrix(5,5) = dummy_matrix(5,5);
%                     P_post_matrix(6,6) = dummy_matrix(6,6);
%                     P_post_matrix(7,7) = dummy_matrix(7,7);
%                     P_post_matrix(8,8) = dummy_matrix(8,8);
%                     P_post_matrix(9,9) = dummy_matrix(9,9);


                    KF_n_phi(te) = (phi(te)- x_post_vector(1))*180/pi;
                    KF_n_lambda(te) = (lambda(te)- x_post_vector(2))*180/pi;
                    KF_n_alt(te) = height(te)- x_post_vector(3);
                    KF_n_ve(te) = v_e(te)- x_post_vector(4);
                    KF_n_vn(te) = v_n(te)- x_post_vector(5);
                    KF_n_vu(te) = v_u(te)- x_post_vector(6);
                    KF_n_Azi(te) = (az- x_post_vector(7))*180/pi;
                    
                    
                    phi(te)=KF_n_phi(te)*pi/180;
                    lambda(te)=KF_n_lambda(te)*pi/180;
                    height(te)=KF_n_alt(te);
                    v_e(te)=KF_n_ve(te);
                    v_n(te)=KF_n_vn(te);
                    v_u(te)=KF_n_vu(te); 
                    Azi(te)=KF_n_Azi(te); 
                   
                    
                    g = a1*(1+a2*sin(phi(te))*sin(phi(te))+a3*sin(phi(te))*sin(phi(te))*sin(phi(te))*sin(phi(te)))+(a4+a5*sin(phi(1))*sin(phi(1)))*height(1)+a6*height(1)*height(1);  % at time 490977.001
                     R_N = a./sqrt(1-e2*sin(phi(te)).*sin(phi(te)));% Normal radius
                     R_M = (a*(1-e2))./((1-e2*sin(phi(te)).*sin(phi(te))).^(1.5));% Meridian radius
                    
                    err_cl_phi(te-1)=x_post_vector(1);
                    err_cl_lambda(te-1)=x_post_vector(2);
                    err_cl_alt(te-1)=x_post_vector(3);
                    err_cl_ve(te-1)=x_post_vector(4);
                    err_cl_vn(te-1)=x_post_vector(5);
                    err_cl_vu(te-1)=x_post_vector(6);
                    err_cl_azi(te-1)=x_post_vector(7);
                    err_cl_aodo(te-1)=x_post_vector(8);
                    err_cl_wz(te-1)=x_post_vector(9);





                 end
               
                KF_n_cl_phi = KF_n_phi;
                KF_n_cl_lambda = KF_n_lambda;
                KF_n_cl_alt = KF_n_alt;
                KF_n_cl_ve = KF_n_ve;
                KF_n_cl_vn = KF_n_vn;
                KF_n_cl_vu = KF_n_vu;
                KF_n_cl_Azi= KF_n_Azi;
                

                KF_n_cl_phi(1) = phi(1)*180/pi;
                KF_n_cl_lambda(1) = lambda(1)*180/pi;
                KF_n_cl_alt(1) = height(1);
                KF_n_cl_ve(1) = v_e(1);
                KF_n_cl_vn(1) = v_n(1);
                KF_n_cl_vu(1) = v_u(1);
                KF_n_cl_Azi(1)= Azi(1);
 
   



   KF_n_cl_azi_err=KF_n_cl_Azi-(ins_PVA.INS_Azi(1+x_shift:plot_len+x_shift))';
   KF_n_ol_azi_err=KF_n_ol_Azi-(ins_PVA.INS_Azi(1+x_shift:plot_len+x_shift))';
   n_ol_azi_err=n_ol_Azi-(ins_PVA.INS_Azi(1+x_shift:plot_len+x_shift))';
   
   for te=1:plot_len;
       if KF_n_cl_azi_err(te)>270
           KF_n_cl_azi_err(te)=KF_n_cl_azi_err(te)-360;
       elseif KF_n_cl_azi_err(te)<-270
           KF_n_cl_azi_err(te)=KF_n_cl_azi_err(te)+360;
       end
       
       if KF_n_ol_azi_err(te)>270
           KF_n_ol_azi_err(te)=KF_n_ol_azi_err(te)-360;
       elseif KF_n_ol_azi_err(te)<-270
           KF_n_ol_azi_err(te)=KF_n_ol_azi_err(te)+360;
       end
       
       if (n_ol_azi_err(te) > 270)
          n_ol_azi_err(te)= n_ol_azi_err(te)-360;
      end
      if (n_ol_azi_err(te) <-270)
           n_ol_azi_err(te)= n_ol_azi_err(te)+360;
      end
%        
   end
       
   
  hh=figure;
   subplot(3,1,1);
   title('Attitude error plot ','fontweight','bold','fontsize',12);
  plot(KF_n_cl_phi-(ins_PVA.INS_Lat(1+x_shift:plot_len+x_shift))','r','LineWidth',2); hold on; plot(KF_n_ol_phi-(ins_PVA.INS_Lat(1+x_shift:plot_len+x_shift))','b','LineWidth',2); %hold on; plot(ins_PVA.INS_Pitch(1+x_shift:plot_len+x_shift),'g');
  grid on;
%   lg=legend('Latitude error MEMS IMU','Latitude error Tactical IMU');
%   gt1=findobj(lg,'type','text');
%   set(gt1,'fontname','--','fontweight','bold');
  
  xlabel('time (seconds)','fontweight','bold','fontsize',10);
  ylabel('Latitude error (degrees)','fontweight','bold','fontsize',10);
  
  subplot(3,1,2);
  plot(KF_n_cl_lambda-(ins_PVA.INS_Long(1+x_shift:plot_len+x_shift))','r','LineWidth',2); hold on; plot(KF_n_ol_lambda-(ins_PVA.INS_Long(1+x_shift:plot_len+x_shift))','b','LineWidth',2); %hold on; plot(ins_PVA.INS_Pitch(1+x_shift:plot_len+x_shift),'g');
  grid on;
%   lg=legend('Longitude error MEMS IMU','Longitude error Tactical IMU');
%   gt1=findobj(lg,'type','text');
%   set(gt1,'fontname','--','fontweight','bold');
  
  xlabel('time (seconds)','fontweight','bold','fontsize',10);
  ylabel('Longitude error (degrees)','fontweight','bold','fontsize',10);
  
  subplot(3,1,3);
  plot(KF_n_cl_alt-(ins_PVA.INS_Alt(1+x_shift:plot_len+x_shift))','r','LineWidth',2); hold on; plot(KF_n_ol_alt-(ins_PVA.INS_Alt(1+x_shift:plot_len+x_shift))','b','LineWidth',2);
  grid on;
%   lg=legend('Height error MEMS IMU','Height error Tactical IMU');
%   gt1=findobj(lg,'type','text');
%   set(gt1,'fontname','--','fontweight','bold');
  
  xlabel('time (seconds)','fontweight','bold','fontsize',10);
  ylabel('Height error (m)','fontweight','bold','fontsize',10);
  print(hh,'-djpeg','-r500','position_error');
%   
%   hh=figure;
%    subplot(3,1,1);
%    title('Attitude error plot ','fontweight','bold','fontsize',12);
%   plot(x_v_e-(ins_PVA.INS_ve(1+x_shift:plot_len+x_shift))','r','LineWidth',2); hold on; plot(n_v_e-(ins_PVA.INS_ve(1+x_shift:plot_len+x_shift))','b','LineWidth',2); %hold on; plot(ins_PVA.INS_Pitch(1+x_shift:plot_len+x_shift),'g');
%   grid on;
% %   lg=legend('Velocity East error MEMS IMU','Velocity East error Tactical IMU');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
% %   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Velocity East error (m/s)','fontweight','bold','fontsize',10);
%   
%   subplot(3,1,2);
%   plot(x_v_n-(ins_PVA.INS_vn(1+x_shift:plot_len+x_shift))','r','LineWidth',2); hold on; plot(n_v_n-(ins_PVA.INS_vn(1+x_shift:plot_len+x_shift))','b','LineWidth',2); %hold on; plot(ins_PVA.INS_Pitch(1+x_shift:plot_len+x_shift),'g');
%   grid on;
% %   lg=legend('Velocity North error MEMS IMU','Velocity North error Tactical IMU');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
% %   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Velocity North error (m/s)','fontweight','bold','fontsize',10);
%   
%   subplot(3,1,3);
%   plot(x_v_u-(ins_PVA.INS_vu(1+x_shift:plot_len+x_shift))','r','LineWidth',2); hold on; plot(n_v_u-(ins_PVA.INS_vu(1+x_shift:plot_len+x_shift))','b','LineWidth',2);
%   grid on;
% %   lg=legend('Velocity Up error MEMS IMU','Velocity Up error Tactical IMU');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
%   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Velocity Up error (m/s)','fontweight','bold','fontsize',10);
% print(hh,'-djpeg','-r500','velocity_error');   

%hh=figure;
%    subplot(3,1,1);
%    %title('Attitude error plot ','fontweight','bold','fontsize',12);
%   plot(x_pitch-(ins_PVA.INS_Pitch(1+x_shift:plot_len+x_shift))','r','LineWidth',2); hold on; plot(n_pitch-(ins_PVA.INS_Pitch(1+x_shift:plot_len+x_shift))','b','LineWidth',2); %hold on; plot(ins_PVA.INS_Pitch(1+x_shift:plot_len+x_shift),'g');
%   grid on;
% %   lg=legend('Pitch error MEMS IMU','Pitch error Tactical IMU');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
%   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Pitch error (degrees)','fontweight','bold','fontsize',10);
%   
%   subplot(3,1,2);
%   plot(x_roll-(ins_PVA.INS_Roll(1+x_shift:plot_len+x_shift))','r','LineWidth',2); hold on; plot(n_roll-(ins_PVA.INS_Roll(1+x_shift:plot_len+x_shift))','b','LineWidth',2); %hold on; plot(ins_PVA.INS_Pitch(1+x_shift:plot_len+x_shift),'g');
%   grid on;
% %   lg=legend('Roll error MEMS IMU','Roll error Tactical IMU');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
% %   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Roll error (degrees)','fontweight','bold','fontsize',10);
%   
%   subplot(3,1,3);
figure;
  plot(KF_n_ol_azi_err,'r','LineWidth',2); hold on; plot(KF_n_cl_azi_err,'b','LineWidth',2); hold on; plot(n_ol_azi_err,'g','LineWidth',2);
  grid on;
  lg=legend('KF_n_ol_azi_err','KF_n_cl_azi_err','n_ol_azi_err');
%   gt1=findobj(lg,'type','text');
%   set(gt1,'fontname','--','fontweight','bold');
  
  xlabel('time (seconds)','fontweight','bold','fontsize',10);
  ylabel('Azimuth error (degrees)','fontweight','bold','fontsize',10);  
  %print(hh,'-djpeg','-r500','attitude_error');


 
% hh=figure;
%    subplot(3,1,1);
%   plot(x_pitch,'r','LineWidth',2); hold on; plot(n_pitch,'b','LineWidth',2); hold on; plot(ins_PVA.INS_Pitch(1+x_shift:plot_len+x_shift),'g','LineWidth',2);
%   grid on;
% %   lg=legend('Pitch MEMS IMU','Pitch  Tactical IMU','Pitch Reference');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
%   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Pitch  (degrees)','fontweight','bold','fontsize',10);
%   
%   subplot(3,1,2);
%   plot(x_roll,'r','LineWidth',2); hold on; plot(n_roll,'b','LineWidth',2); hold on; plot(ins_PVA.INS_Roll(1+x_shift:plot_len+x_shift),'g','LineWidth',2);
%   grid on;
% %   lg=legend('Roll MEMS IMU','Roll  Tactical IMU','Roll Reference');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
%   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Roll  (degrees)','fontweight','bold','fontsize',10);
%   
%   subplot(3,1,3);
%   plot(x_Azi,'r','LineWidth',2); hold on; plot(n_Azi,'b','LineWidth',2); hold on; plot(ins_PVA.INS_Azi(1+x_shift:plot_len+x_shift),'g','LineWidth',2);
%   grid on;
% %   lg=legend('Azimuth MEMS IMU','Azimuth  Tactical IMU','Azimuth Reference');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
%   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Azimuth  (degrees)','fontweight','bold','fontsize',10);
% print(hh,'-djpeg','-r500','Attitude');

% hh=figure;
%    subplot(3,1,1);
%   plot(x_v_e,'r','LineWidth',2); hold on; plot(n_v_e,'b','LineWidth',2); hold on; plot(ins_PVA.INS_ve(1+x_shift:plot_len+x_shift),'g','LineWidth',2);
%   grid on;
% %   lg=legend('East Velocity MEMS IMU','East Velocity  Tactical IMU','East Velocity Reference');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
%   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('East Velocity (m/s)','fontweight','bold','fontsize',10);
%   
%   subplot(3,1,2);
%   plot(x_v_n,'r','LineWidth',2); hold on; plot(n_v_n,'b','LineWidth',2); hold on; plot(ins_PVA.INS_vn(1+x_shift:plot_len+x_shift),'g','LineWidth',2);
%   grid on;
% %   lg=legend('North Velocity MEMS IMU','North Velocity  Tactical IMU','North Velocity Reference');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
% %   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('North Velocity  (m/s)','fontweight','bold','fontsize',10);
%   
%   subplot(3,1,3);
%   plot(x_v_u,'r','LineWidth',2); hold on; plot(n_v_u,'b','LineWidth',2); hold on; plot(ins_PVA.INS_vu(1+x_shift:plot_len+x_shift),'g','LineWidth',2);
%   grid on;
% %   lg=legend('Up Velocity MEMS IMU','Up Velocity  Tactical IMU','Up Velocity Reference');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
%   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Up Velocity  (m/s)','fontweight','bold','fontsize',10);
%   print(hh,'-djpeg','-r500','Velocity');



% hh=figure;
%    subplot(3,1,1);
%   plot(x_phi,'r','LineWidth',2); hold on; plot(n_phi,'b','LineWidth',2); hold on; plot(ins_PVA.INS_Lat(1+x_shift:plot_len+x_shift),'g','LineWidth',2);
%   grid on;
% %   lg=legend('Lattitude MEMS IMU','Lattitude  Tactical IMU','Lattitude Reference');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
% %   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Lattitude (degrees)','fontweight','bold','fontsize',10);
%   
%   subplot(3,1,2);
%   plot(x_lambda,'r','LineWidth',2); hold on; plot(n_lambda,'b','LineWidth',2); hold on; plot(ins_PVA.INS_Long(1+x_shift:plot_len+x_shift),'g','LineWidth',2);
%   grid on;
% %   lg=legend('Longitude MEMS IMU','Longitude  Tactical IMU','Longitude Reference');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
% %   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Longitude  (degrees)','fontweight','bold','fontsize',10);
%   
%   subplot(3,1,3);
%   plot(x_height,'r','LineWidth',2); hold on; plot(n_height,'b','LineWidth',2); hold on; plot(ins_PVA.INS_Alt(1+x_shift:plot_len+x_shift),'g','LineWidth',2);
%   grid on;
% %   lg=legend('Altitude MEMS IMU','Altitude  Tactical IMU','Up Velocity Reference');
% %   gt1=findobj(lg,'type','text');
% %   set(gt1,'fontname','--','fontweight','bold');
%   
%   xlabel('time (seconds)','fontweight','bold','fontsize',10);
%   ylabel('Altitude  (m)','fontweight','bold','fontsize',10);
%   print(hh,'-djpeg','-r500','position');
%   
  
  
 hh=figure;
 plot(KF_n_cl_lambda, KF_n_cl_phi, 'rO--','LineWidth',2); hold on; plot(KF_n_ol_lambda, KF_n_ol_phi, 'b*--','LineWidth',2); hold on; plot(ins_PVA.INS_Long(1+x_shift:plot_len+x_shift),ins_PVA.INS_Lat(1+x_shift:plot_len+x_shift),'g--','LineWidth',3); 
 %hold on;  plot3(novatel_gps_pos.GP_Long,novatel_gps_pos.GP_Lat, novatel_gps_pos.GP_Alt,'b--','LineWidth',3);
  grid on;
  lg=legend('closed','open','reference');
  gt1=findobj(lg,'type','text');
  set(gt1,'fontname','--','fontweight','bold');
    
  xlabel('Longitude (Degrees)','fontweight','bold','fontsize',12);
  ylabel('Latitude (Degrees)','fontweight','bold','fontsize',12);
 %zlabel('Height (meters)','fontweight','bold','fontsize',12);
  title('Position plot: 3D RISS Mechanizations ','fontweight','bold','fontsize',12);
% print(hh,'-djpeg','-r500','pos_plot_3d_riss');
% % 
%  
%  


 

 
 
 