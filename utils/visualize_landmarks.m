function visualize_landmarks(data, landmarks_est)
% VISUALIZE_LANDMARKS
% Visualizes:
%   - Ground truth landmarks
%   - Odometry and ground truth trajectories
%   - (Optional) Triangulated landmarks if provided
% Inputs:
%   data          - struct with fields:
%       .world         - [N x 4] landmark GT: [id, x, y, z]
%       .trajectory    - [N x 7] trajectory: [id, odom(1:3), gt(1:3)]
%   landmarks_est - struct array with .id and .pos (can be empty [])

if nargin < 2
    landmarks_est = []; % No triangulated landmarks provided
end

figure; hold on; grid on;
title('SLAM Visualization');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal;

% --- Plot ground truth landmarks ---
gt_ids = data.world(:,1);
gt_pos = data.world(:,2:4);
scatter3(gt_pos(:,1), gt_pos(:,2), gt_pos(:,3), 5, 'g', 'filled', 'DisplayName', 'GT Landmarks');

% --- Plot trajectories ---
odom = data.trajectory(:,2:3);
gt   = data.trajectory(:,5:6);
plot3(odom(:,1), odom(:,2), zeros(size(odom,1),1), 'b--', 'DisplayName', 'Odometry Trajectory');
plot3(gt(:,1),   gt(:,2),   zeros(size(gt,1),1),   'k-', 'DisplayName', 'GT Trajectory');

% --- Optional: Plot triangulated landmarks ---
if ~isempty(landmarks_est)
    est_ids = [landmarks_est.id];
    est_pos = cell2mat(arrayfun(@(l) l.pos(:), landmarks_est, 'UniformOutput', false));
    est_pos = reshape(est_pos, 3, [])';

    % Match common landmark IDs
    [common_ids, est_idx, gt_idx] = intersect(est_ids, gt_ids);
    matched_est = est_pos(est_idx, :);
    matched_gt  = gt_pos(gt_idx, :);

    scatter3(matched_est(:,1), matched_est(:,2), matched_est(:,3), 5, 'r', 'DisplayName', 'Triangulated');

    % % Optional: lines connecting GT and estimated
    % for i = 1:length(common_ids)
    %     line([matched_gt(i,1), matched_est(i,1)], ...
    %          [matched_gt(i,2), matched_est(i,2)], ...
    %          [matched_gt(i,3), matched_est(i,3)], ...
    %          'Color', [0.5 0.5 0.5], 'LineStyle', ':');
    % end
end

legend;
drawnow;
fprintf('[DEBUG] Press any key to continue after reviewing plots...\n');
pause;
end
