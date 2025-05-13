function plot_ba_stats(dx_norm, chi_total, chi_r, chi_p)
% Plots Chi² and ‖dx‖ evolution over BA iterations
%
% Inputs:
%   dx_norm     - step norm per iteration
%   chi_total   - total chi² per iteration
%   chi_r       - pose-related chi² per iteration
%   chi_p       - projection-related chi² per iteration

figure('Name', 'BA Optimization Stats', 'NumberTitle', 'off');
set(gcf, 'Position', [100, 100, 1000, 400]);

% --- Chi² subplot ---
subplot(1,2,1); hold on; grid on;
plot(chi_total, 'b-', 'LineWidth', 2, 'DisplayName', 'Total χ²');
plot(chi_r, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Pose χ²');
plot(chi_p, 'g:', 'LineWidth', 1.5, 'DisplayName', 'Proj. χ²');
title('Chi² over Iterations');
xlabel('Iteration'); ylabel('Chi²');
legend('show');

% --- Step norm subplot ---
subplot(1,2,2); hold on; grid on;
plot(dx_norm, 'm-', 'LineWidth', 2);
title('Step Norm ‖dx‖ over Iterations');
xlabel('Iteration'); ylabel('‖dx‖');

sgtitle('Bundle Adjustment Statistics');
end
