function visualize_landmarks(landmarks_est, data)
% VISUALIZE_LANDMARKS Plots estimated landmarks vs ground truth
% Inputs:
%   landmarks_est - struct array with .id and .pos
%   data          - dataset struct (must contain data.world)



% Extract estimated positions
est_ids = [landmarks_est.id];
est_pos = cell2mat(arrayfun(@(l) l.pos(:), landmarks_est, 'UniformOutput', false));
est_pos = reshape(est_pos, 3, [])';

% Ground truth
gt_ids = data.world(:,1);
gt_pos = data.world(:,2:4);

% Match common landmark IDs
[common_ids, est_idx, gt_idx] = intersect(est_ids, gt_ids);
matched_est = est_pos(est_idx, :);
matched_gt = gt_pos(gt_idx, :);

% Plot 3D scatter
figure; hold on; grid on;
scatter3(matched_gt(:,1), matched_gt(:,2), matched_gt(:,3), 20, 'g', 'filled');
scatter3(matched_est(:,1), matched_est(:,2), matched_est(:,3), 20, 'r');
legend('Ground Truth', 'Estimated');
xlabel('X'); ylabel('Y'); zlabel('Z');
title(sprintf('Landmark Map Comparison (%d matched)', length(common_ids)));
axis equal;
drawnow;

% Optional: show connection lines
% for i = 1:length(common_ids)
%     line([matched_gt(i,1), matched_est(i,1)], ...
%          [matched_gt(i,2), matched_est(i,2)], ...
%          [matched_gt(i,3), matched_est(i,3)], ...
%          'Color', [0.5 0.5 0.5], 'LineStyle', ':');
% end 
fprintf('[DEBUG] Press any key to continue after reviewing plots...\n');
pause;
end
