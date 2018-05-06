%% Initializations

%prepare simulation
close all;
clear;
clc;

%camera intrinsic and extrinsic parameters
width = 704; height = 472;
fx = width; fy = width; cx = width/2; cy = height/2; %no distortion
K = [cx -fx 0;cy 0 -fy;1 0 0];
t = [1.5 3 2]';
yaw = -90; roll = 45;
Ryaw = [cosd(yaw) -sind(yaw) 0; sind(yaw) cosd(yaw) 0; 0 0 1];
Rroll = [1 0 0; 0 cosd(roll) -sind(roll); 0 sind(roll) cosd(roll);]; 
R = Rroll * Ryaw;

%marker geometry
%x_beacon = [0.0212,0.0212,0.075; 0.0,0.20,0.075; 0.20,0.0,0.075; 0.0,0.0,0.21;]'; %new one
% x_beacon = [0.22,0.0,0.0; 0.0,0.22,0.0; 0.08,0.08,0.00; 0.0,0.0,0.25;]'; %old one
x_beacon = [0.20,0.0,0.0; 0.0,0.13,0.0; 0.10,0.10,0.00; 0.0,-0.02,0.00;]'; %testing ones
% x_beacon = [0.00,-0.10,0.00; 0.0,0.15,0.00; 0.0,0.0,0.05; 0.20,0.0,0.00]';

%marker position
t_marker = [1.5 1.8 0.2]';
yaw_marker = pi/3;
Ryaw = [cos(yaw_marker) -sin(yaw_marker) 0; sin(yaw_marker) cos(yaw_marker) 0; 0 0 1];
R_marker = Ryaw;
vyaw_marker = 0.4189;

%% Filter

%noise parameters
R_noise = [4 0;0 4];
Q_noise = eye(12)*0.01;

%initializations
P = eye(12);
x = [reshape(R_marker', 9, 1);t_marker] + 1*sqrt(Q_noise)*randn(12, 1);
x

for k = 1:1000

    %predict
    %state mantains (random walk)
    P = P + 1 * Q_noise;
    
    %get measurements
    K1 = inv(K);
    x_beacon_l = t_marker + R_marker * x_beacon;
    x_beacon_p = K * x_beacon_l;
    x_beacon_p = x_beacon_p ./ repmat(x_beacon_p(3,:), 3, 1);
    uv_l = K1 * [x_beacon_p(1:2, :);ones(1,size(x_beacon_p, 2))];
    x_beacon_p(1:2, :) = x_beacon_p(1:2, :) + 2*randn(2, size(x_beacon_p, 2));
    u = x_beacon_p(1, :);
    v = x_beacon_p(2, :);
    uv = K1 * [u;v;ones(1,length(u))];
    u = uv(2, :);
    v = uv(3, :);

    %update
    z = []; H = []; V = [];
    for l = 1:3
        Hl = [ (u(l)*x_beacon(1:3,l)') -(x_beacon(1:3,l)')     zeros(1, 3)    u(l) -1  0 ;
              (v(l)*x_beacon(1:3,l)')      zeros(1, 3)    -(x_beacon(1:3,l)') v(l)  0 -1;];
        H = cat(1, H, Hl);

        Vl = K1(2:3,1:2)*[ (x_beacon(1:3,l)')*x(1:3) + x(10)            0           ;
                                0                 (x_beacon(1:3,l)')*x(1:3) + x(10);];
        V = cat(1, V, Vl);
        
        z = cat(1, z, zeros(2, 1));
    end
    R_test = 0.01*eye(size(V, 1));
    K_gain = P*H'*inv(1*H*P*H' + R_test + 0*V*R_noise*V');
    
    x = x + K_gain*(z - H*x);
    P = (eye(12) - K_gain*H)*P;
    
    %estimate projection
    for k = 1:1
        g = [x(1)*x(1) + x(4)*x(4) + x(7)*x(7);
             x(2)*x(2) + x(5)*x(5) + x(8)*x(8);
             x(3)*x(3) + x(6)*x(6) + x(9)*x(9);];
        D = 2 * [x(1) 0 0 x(4) 0 0 x(7) 0 0 0 0 0;
                 0 x(2) 0 0 x(5) 0 0 x(8) 0 0 0 0;
                 0 0 x(3) 0 0 x(6) 0 0 x(9) 0 0 0;];
        d = ones(3, 1) - g + D*x;
        x = x - P*D'*inv(D*P*D')*(D*x - d);
    end

end

%problem is that this is perfect measurement (it does not work with noise on the measurement)

%% Filter2 


%noise parameters
R_noise = [4 0;0 4];
Q_noise = eye(12)*0.05;

%initializations
P = eye(12);
x = [reshape(R_marker', 9, 1);t_marker] + 1*sqrt(Q_noise)*randn(12, 1);
x

for k = 1:100

    %predict
    %state mantains (random walk)
    P = P + 10*Q_noise;
    
    %get measurements
    K1 = inv(K);
    x_beacon_l = t_marker + R_marker * x_beacon;
    x_beacon_p = K * x_beacon_l;
    x_beacon_p = x_beacon_p ./ repmat(x_beacon_p(3,:), 3, 1);
    uv_l = K1 * [x_beacon_p(1:2, :);ones(1,length(u))];
    x_beacon_p(1:2, :) = x_beacon_p(1:2, :) + 2*randn(2, size(x_beacon_p, 2));
    u = x_beacon_p(1, :);
    v = x_beacon_p(2, :);
    uv = K1 * [u;v;ones(1,length(u))];
    u = uv(2, :);
    v = uv(3, :);

    %update
    z = []; H = []; V = []; h = [];
    for l = 1:4
        lambda = x_beacon(1:3,l)'*x(1:3) + x(10);
        lx = x_beacon(1:3,l)'*x(4:6) + x(11);
        ly = x_beacon(1:3,l)'*x(7:9) + x(12);
        Hl = (1/lambda)^2 * ...
             [-lx * x_beacon(1:3,l)' x_beacon(1:3,l)'*lambda zeros(1, 3) -lx lambda 0;
              -ly * x_beacon(1:3,l)' zeros(1, 3) x_beacon(1:3,l)'*lambda -ly 0 lambda;];
        H = cat(1, H, Hl);

        Vl = K1(2:3,1:2)*[ lambda             0;
                              0           lambda;];
        V = cat(1, V, Vl);
        
        z = cat(1, z, [u(l);v(l)]);
        hl = [lx / lambda;
              ly / lambda;];
        h= cat(1, h, hl);
        
    end
    K_gain = P*H'*inv(H*P*H' + V*R_noise*V');
    
%     H*x
    
    x = x + K_gain*(z - h);
    P = (eye(12) - K_gain*H)*P;
    
    %estimate projection
    g = [x(1)*x(1) + x(4)*x(4) + x(7)*x(7);
         x(2)*x(2) + x(5)*x(5) + x(8)*x(8);
         x(3)*x(3) + x(6)*x(6) + x(9)*x(9);];
    D = 2 * [x(1) 0 0 x(4) 0 0 x(7) 0 0 0 0 0;
             0 x(2) 0 0 x(5) 0 0 x(8) 0 0 0 0;
             0 0 x(3) 0 0 x(6) 0 0 x(9) 0 0 0;];
    d = ones(3, 1) - g + D*x;
    x = x - P*D'*inv(D*P*D')*(D*x - d);

end

%% filter3

%noise parameters
R_noise = [4 0;0 4];
Q_noise = eye(12)*0.05;

%initializations
P = 0.1*eye(12);
x = [reshape(R_marker', 9, 1);t_marker] + 1*sqrt(Q_noise)*randn(12, 1);
x

for k = 1:10

    %predict
    %state mantains (random walk)
    P = P + Q_noise;
    
    %get measurements
    K1 = inv(K);
    x_beacon_l = t_marker + R_marker * x_beacon;
    x_beacon_p = K * x_beacon_l;
    x_beacon_p = x_beacon_p ./ repmat(x_beacon_p(3,:), 3, 1);
    uv_l = K1 * [x_beacon_p(1:2, :);ones(1,length(u))];
    x_beacon_p(1:2, :) = x_beacon_p(1:2, :) + 0.5*randn(2, size(x_beacon_p, 2));
    u = x_beacon_p(1, :);
    v = x_beacon_p(2, :);
    uv = K1 * [u;v;ones(1,length(u))];
    u = uv(2, :);
    v = uv(3, :);

    %update
    z = []; H = []; V = []; h = [];
    for l = 1:4
        
        A = [-(u(l)*x_beacon(1:3,l)') (x_beacon(1:3,l)')    zeros(1, 3);
             -(v(l)*x_beacon(1:3,l)')     zeros(1, 3)    (x_beacon(1:3,l)');];
        A1 = [x_beacon(1:3,l)' zeros(1, 6);
              x_beacon(1:3,l)' zeros(1, 6);];
        B = [u(l) -1 0;v(l) 0 -1];
        
        
        
        Hl = B;
        H = cat(1, H, Hl);
        
        Vl = K1(2:3,1:2)*[ (x_beacon(1:3,l)')*x(1:3) + x(10)            0           ;
                                0                 (x_beacon(1:3,l)')*x(1:3) + x(10);];
        Vl = [Vl (-A + A1)];
        V = cat(1, V, Vl);
        
        h = cat(1, h, B*x(10:12));
        z = cat(1, z, A*x(1:9));
    end
    Rt = [R_noise     zeros(2, 9)
          zeros(9, 2) P(1:9, 1:9)];
    K_gain = P(10:12,10:12)*H'*inv(H*P(10:12,10:12)*H' + V*Rt*V');
    
    x(10:12) = x(10:12) + K_gain*(z - h);
    P(10:12,10:12) = (eye(3) - K_gain*H)*P(10:12,10:12);

end
