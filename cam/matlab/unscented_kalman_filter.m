function [x_out, Pxx_out] = unscented_kalman_filter(x, Pxx, f, p, y)

    %get dimension of state
    L = length(x);
    
    %compute spread parameter
    alpha = 0.5;
    beta = 2;
    lambda = alpha*alpha*L - L;
    
    %compute mean weights
    Wm = lambda/(L + lambda);
    for k = 1:2*L
        Wm = cat(1, Wm, 1/(2*(L + lambda)));
    end
    
    %compute covariance weights
    Wc = lambda/(L + lambda) + (1 - alpha*alpha + beta);
    for k = 1:2*L
        Wc = cat(1, Wc, 1/(2*(L + lambda)));
    end
    
    %compute sigma points
    x_sigma = x;
    tic
    for k = 1:L
        Pxx_sigma = sqrtm((L + lambda)*Pxx);
        x_sigma = cat(2, x_sigma, x + Pxx_sigma(k, :)');
    end
    for k = 1:L
        Pxx_sigma = sqrtm((L + lambda)*Pxx);
        x_sigma = cat(2, x_sigma, x - Pxx_sigma(k, :)');
    end
    toc

    %propagate sigma points
    y_sigma = [];
    for k= 1:2*L + 1
        y_sigma = cat(2, y_sigma, f(x_sigma(:, k), p));
    end
    
    %compute the transformed statistics (mean)
    y_sigma_mean = zeros(size(y_sigma, 1), 1);
    x_sigma_mean = zeros(size(x_sigma, 1), 1);
    for k = 1:2*L + 1
        x_sigma_mean = x_sigma_mean + Wm(k)*x_sigma(:, k);
        y_sigma_mean = y_sigma_mean + Wm(k)*y_sigma(:, k);
    end
    
    %compute the transformed statistics (covariance)
    dx_sigma = x_sigma - repmat(x_sigma_mean, 1, 2*L + 1);
    dy_sigma = y_sigma - repmat(y_sigma_mean, 1, 2*L + 1);
    Pyy = zeros(size(dy_sigma, 1), size(dy_sigma, 1));
    Pxy = zeros(size(dx_sigma, 1), size(dy_sigma, 1));
    Pxx_new = zeros(size(dx_sigma, 1), size(dx_sigma, 1));
    for k = 1:2*L + 1
        Pyy = Pyy + Wc(k)*dy_sigma(:, k)*dy_sigma(:, k)';
        Pxy = Pxy + Wc(k)*dx_sigma(:, k)*dy_sigma(:, k)';
        Pxx_new = Pxx_new + Wc(k)*dx_sigma(:, k)*dx_sigma(:, k)';
    end
    
    %apply the unscented Kalman fiter equation
    K = Pxy * inv(Pyy);
    x_out = x_sigma_mean + K*(y - y_sigma_mean);
    Pxx_out = Pxx_new - K*Pyy*K';
    
end