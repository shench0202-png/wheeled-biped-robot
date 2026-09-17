%% LQR_K_vary_l
% Calculate LQR gain K over different leg lengths, then print the
% piecewise-linear equations between 5 selected points.
clear; clc; close all;

% ==========================================
% 1. Model parameters
% ==========================================
M = 0.15;      % Wheel/base equivalent mass (kg)

m_body = 0.2222 + 0.1546 + 0.0463 + 0.025;
m_legs = 0.2;
m_mass_reduction = 0.222;
m = m_body + m_legs - m_mass_reduction;  % Body/leg equivalent pendulum mass (kg)

g = 9.81;          % Gravity (m/s^2)
R_wheel = 0.03;    % Wheel radius (m)

% LQR weights for state x = [x, x_dot, theta, theta_dot]
Q = diag([1, 1, 8, 1]);
R_lqr = 5;         % Control effort weight

% Leg length range (mm)
l_mm_range = 60:1:156.38;
num_points = length(l_mm_range);

K_results = zeros(num_points, 4);

% ==========================================
% 2. Calculate LQR K for each leg length
% ==========================================
for i = 1:num_points
    l = l_mm_range(i) / 1000;  % mm to m

    % Point-mass inverted pendulum approximation.
    % Jb can be replaced by a measured/CAD pitch-axis inertia later.
    Jb = 0;

    Meq = M + m;
    Jeq = Jb + m * l^2;
    Delta = Jeq * Meq - (m * l)^2;

    a23 = -(m^2 * l^2 * g) / Delta;
    a43 = (Meq * m * l * g) / Delta;

    A = [0, 1, 0, 0;
         0, 0, a23, 0;
         0, 0, 0, 1;
         0, 0, a43, 0];

    b2 = Jeq / (R_wheel * Delta);
    b4 = -(m * l) / (R_wheel * Delta);

    B = [0; b2; 0; b4];

    [K, ~, ~] = lqr(A, B, Q, R_lqr);
    K_results(i, :) = K;
end

% ==========================================
% 3. Select 5 points and print equations
% ==========================================
idx_5pts = round(linspace(1, num_points, 5));
l_5pts = l_mm_range(idx_5pts);
K_5pts = K_results(idx_5pts, :);

fprintf('\nSelected 5 LQR gain points:\n');
fprintf('Length_mm\tK1\t\tK2\t\tK3\t\tK4\n');
for i = 1:length(l_5pts)
    fprintf('%8.2f\t% .6f\t% .6f\t% .6f\t% .6f\n', ...
        l_5pts(i), K_5pts(i, 1), K_5pts(i, 2), K_5pts(i, 3), K_5pts(i, 4));
end

fprintf('\nPiecewise-linear equations between selected points:\n');
fprintf('Use l_mm as leg length in mm. Each segment is K = a*l_mm + b.\n\n');

piecewise_eq = struct();

for k_idx = 1:4
    fprintf('K%d equations:\n', k_idx);
    piecewise_eq(k_idx).name = sprintf('K%d', k_idx);
    piecewise_eq(k_idx).segments = zeros(length(l_5pts) - 1, 4);

    for seg_idx = 1:(length(l_5pts) - 1)
        x1 = l_5pts(seg_idx);
        x2 = l_5pts(seg_idx + 1);
        y1 = K_5pts(seg_idx, k_idx);
        y2 = K_5pts(seg_idx + 1, k_idx);

        a = (y2 - y1) / (x2 - x1);
        b = y1 - a * x1;

        piecewise_eq(k_idx).segments(seg_idx, :) = [x1, x2, a, b];

        fprintf('  %.2f <= l_mm <= %.2f: K%d = %.10f*l_mm %+.10f\n', ...
            x1, x2, k_idx, a, b);
    end
    fprintf('\n');
end

% ==========================================
% 4. Plot K curves and selected points
% ==========================================
figure('Name', 'LQR K values vs Length (Point Mass Model)', ...
    'Position', [100, 100, 900, 700]);

titles = {'K_1 (x feedback gain)', ...
          'K_2 (x_dot feedback gain)', ...
          'K_3 (theta feedback gain)', ...
          'K_4 (theta_dot feedback gain)'};
y_labels = {'Gain Value', 'Gain Value', 'Gain Value', 'Gain Value'};
colors = {'b', 'r', 'g', 'm'};

for k_idx = 1:4
    subplot(2, 2, k_idx);
    plot(l_mm_range, K_results(:, k_idx), colors{k_idx}, 'LineWidth', 2);
    hold on;
    plot(l_5pts, K_5pts(:, k_idx), 'ko', 'MarkerFaceColor', 'k');

    for j = 1:length(l_5pts)
        text(l_5pts(j), K_5pts(j, k_idx), ...
            sprintf('  %.2f', K_5pts(j, k_idx)), ...
            'FontSize', 8, 'VerticalAlignment', 'bottom');
    end

    title(titles{k_idx});
    xlabel('Leg length l (mm)');
    ylabel(y_labels{k_idx});
    grid on;
end

sgtitle('LQR gain K vs leg length with 5-point piecewise equations');
