function rmse = evaluate_map(landmarks_est, data)
% EVALUATE_MAP Computes RMSE between estimated and ground truth 3D points
% Inputs:
%   landmarks_est - struct array with .id and .pos
%   data          - struct containing .world (GT landmark positions)
% Output:
%   rmse - scalar RMSE error over all matched landmarks

% Extract ground truth
world_gt = data.world;
gt_ids = world_gt(:,1);
gt_pos = world_gt(:,2:4);

% Extract estimates
est_ids = [landmarks_est.id];
est_pos = cell2mat(arrayfun(@(l) l.pos(:), landmarks_est, 'UniformOutput', false));
est_pos = reshape(est_pos, 3, [])';

% Match IDs
[common_ids, est_idx, gt_idx] = intersect(est_ids, gt_ids);
matched_est = est_pos(est_idx, :);
matched_gt = gt_pos(gt_idx, :);

% Compute RMSE
if isempty(common_ids)
    warning('No common landmark IDs found for evaluation.');
    rmse = NaN;
    return;
end

errors = matched_est - matched_gt;
squared_error = sum(errors.^2, 2);
rmse = sqrt(mean(squared_error));

fprintf('[EVAL] Matched %d landmarks\n', length(common_ids));
fprintf('[EVAL] Landmark RMSE: %.4f units\n', rmse);
end