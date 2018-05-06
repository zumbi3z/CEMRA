%% Exponential map for SE3 (tangent space -> lie algebra -> lie group)

u = [1 0 0]';
w = [1 0 0.7854]';
[chi, V, invV] = exponential_map_SE3(u, w);

%% Log map (lie group -> lie algebra -> tangent space)

[u_new, w_new] = log_map_SE3(chi);

%% Errors expressed in axis-agle format

%example of an axis-angle uncertainty matrix
sP = sqrt(diag([0.1 0.4 0.1]));
us = sP*randn(3, 1000);
figure(1); hold on; h = [];
for k = 1:size(us, 2)
    h = cat(1, h, line([0 us(1, k)], [0 us(2, k)], [0 us(3, k)], 'color', 'r'));
end
axis equal;

%if the uncertainty is diagonal with the same weights, then we have an
%uniform rotation error around zero and with an intensity given by the
%weights

%% Tangent space' uncertainty matrix

P = diag([ [0.1 0.1 0.1] [0.4 0.4 0.4] ]); %translation + axis-angle uncertainty

%% Observation model

%camera intrinsic parameters
W = 640;
H = 480;
fx = W;
fy = W;
cx = W/2;
cy = H/2;
K = [cx fx 0;
     cy 0 fy;];

%camera extrinsic parameters
R_Ci = eye(3);

%3D beacon layout
x_b_jm = [1 0 0]';

%relative pose of the marker expressed in Lie group
R = eye(3);
t = [1 0 0]';
chi = [R t;zeros(1, 3) 1];

%project point into camera (measurement model)
x_b_jm_C = inv(R_Ci) * (chi(1:3, 4) + chi(1:3, 1:3)*x_b_jm);
p = (1/x_b_jm_C(1)) * K * x_b_jm_C;

%% UKF

%real relative pose of marker
R = exponential_map_SO3([0 0 0.2]');
t = [1 0 0]';
chi = [R t;zeros(1, 3) 1];

%predicted relative pose of marker
R_e = exponential_map_SO3([0 0 0.01]');
t_e = [1 0.2 0]';
chi_e = [R_e t_e;zeros(1, 3) 1];


%uncertainty in covariances
P = diag([ [0.02 0.02 0.02] [0.01 0.01 0.01] ]); %translation + axis-angle uncertainty
R_noise = eye(2) * 1; %pixel uncertainty

%compute sqrt of covariances for sampling
sP = sqrt(P);
sR = sqrt(R_noise);

%do a measurement
x_b_jm_C = inv(R_Ci) * (chi(1:3, 4) + chi(1:3, 1:3)*x_b_jm);
p_measured = (1/x_b_jm_C(1)) * K * x_b_jm_C;
n_v = sR * randn(2, 1);
p_measured = p_measured + n_v;

%unscented transform

wm_i = lambda/(L + lambda);
wc_i = lambda/(L + lambda) + (1 - alpha^2 + beta);
Wm_i = 1/(2*(L+lambda));
wc_i = 1/(2*(L+lambda));

%compute number of UKF samples
n = 2*(size(P, 1) + size(R_noise, 1));

%compute samples
samples = [];
for k = 1:n
    
    %sample
    n_x = sP * randn(6, 1);
    n_v = sR * randn(2, 1);
    
    %compute state deviation
    n_chi = [exponential_map_SO3(n_x(4:6)) n_x(1:3); zeros(1, 3) 1];
    chi_e_k = chi_e + n_chi;
    
    %compute measurement with deviation
    x_b_jm_C = inv(R_Ci) * (chi_e_k(1:3, 4) + chi_e_k(1:3, 1:3)*x_b_jm);
    p_k = (1/x_b_jm_C(1)) * K * x_b_jm_C;
    p_k = p_k + n_v;
    
    %save into vector
    samples = cat(2, samples, [n_chi(1:3, 4);log_map_SO3(n_chi(1:3, 1:3));n_v;p_k]);
    
end

%UKF
ps = samples(9:10, :);
ps_m = mean(ps, 2);
ps_spread = ps - repmat(ps_m, 1, size(ps, 2));
Pyy = ps_spread * ps_spread';
alpha = samples(1:8, :);
alpha_m = mean(alpha, 2);
alpha_spread = alpha - repmat(alpha_m, 1, size(alpha, 2));
Pay = alpha_spread*ps_spread';

a = Pay * inv(Pyy) * (p_measured - ps_m);
b = [P zeros(6, 2);zeros(2, 6) R_noise] - Pay * (Pay * inv(Pyy))';

%% UKF - correct

%camera parameters
p.W = 640;
p.H = 480;
p.fx = p.W;
p.fy = p.W;
p.cx = p.W/2;
p.cy = p.H/2;
p.K = [p.cx p.fx 0;
     p.cy 0 p.fy;];
p.R_Ci = eye(3);

%3D beacon layout
% p.x_b_jm = [1 0 0]';
p.x_b_jm = [0.3 0 0; 0 0.3 0]';
% p.x_b_jm = [0.3 0 0; 0 0.3 0; 0 -0.0 0.3]';


%real relative pose of marker
R = exponential_map_SO3([0 0 0.01]');
t = [1 0 0]';
chi = [R t;zeros(1, 3) 1];

%predicted relative pose of marker
R_e = exponential_map_SO3([0.5 0.3 -0.41]');
t_e = [1 0.2 0]';
chi_e = [R_e t_e;zeros(1, 3) 1];

%uncertainty in covariances
P = diag([ 10*[0.02 0.02 0.02] 10*[0.01 0.01 0.01] ]); %translation + axis-angle uncertainty
R_noise = eye(2) * 1; %pixel uncertainty

%compute sqrt of covariances for sampling
sP = sqrt(P);
sR = sqrt(R_noise);

%estimate state
for k = 1:5

    %measure
    p_measured = [];
    for l = 1:size(p.x_b_jm, 2)
        x_b_jm_C = inv(p.R_Ci) * (chi(1:3, 4) + chi(1:3, 1:3)*p.x_b_jm(:, l));
        z = (1/x_b_jm_C(1)) * p.K * x_b_jm_C;
        n_v = sR * randn(2, 1);
        p_measured = cat(1, p_measured, z + n_v);
    end

    % ----- apply UKF -----
    %1 - extend state
    x = [chi_e(1:3, 4);log_map_SO3(chi_e(1:3, 1:3));zeros(2, 1);zeros(2, 1);zeros(2, 1)];
    Pxx = [P zeros(6, 2) zeros(6, 2) zeros(6, 2);
           zeros(2, 6) R_noise zeros(2, 2) zeros(2, 2);
           zeros(2, 6) zeros(2, 2) R_noise zeros(2, 2);
           zeros(2, 6) zeros(2, 2) zeros(2, 2) R_noise];

    %2 - apply UKF
    f = @project_beacon;
    [x_out, Pxx_out] = unscented_kalman_filter(x, Pxx, f, p, p_measured);

    %3 - re-compress state
    chi_e = [exponential_map_SO3(x_out(4:6)) x_out(1:3); zeros(1, 3) 1];
    P = Pxx_out(1:6, 1:6);

    %4 - remove computational errors and avoiding inconsistency
    P = 0.5*P + 0.5*P';
    P = P + 0.01*eye(6);

end

%output
chi_e
