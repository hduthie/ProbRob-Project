function [landmarks, ids] = triangulate_simple(data, selected_ids)
% TRIANGULATE_SIMPLE - DLT triangulation using odometry
% Only triangulates landmarks specified in `selected_ids`.
%
% Inputs:
%   data         - SLAM dataset struct (with .camera, .trajectory, .measurements)
%   selected_ids - vector of landmark IDs to triangulate
%
% Outputs:
%   landmarks - struct array with fields: id, pos, obs
%   ids       - map from landmark_id → index in landmarks

fprintf('[TRIANGULATE_SIMPLE] Starting triangulation (filtered IDs)...\n');

if nargin < 2
    selected_ids = sort(cell2mat(keys(data.measurements)));  % use all if not provided
end

K = data.camera.K;
T_cam = inv(data.camera.cam_transform);
num_poses = size(data.trajectory, 1);
num_landmarks = max(selected_ids) + 1;

D = cell(num_landmarks, 1);
index_vec = zeros(1, num_landmarks);
P_base = K * [eye(3), zeros(3,1)] * T_cam;

% Fill design matrix
for pose_idx = 1:num_poses
    T_robot = se2_to_SE3(data.trajectory(pose_idx, 2:4));
    % T_robot = se2_to_SE3(data.trajectory(pose_idx, 5:7));  % GT pose

    P = P_base * inv(T_robot);

    for i = 1:length(selected_ids)
        landmark_id = selected_ids(i);
        if ~isKey(data.measurements, landmark_id), continue; end
        obs = data.measurements(landmark_id);
        obs_in_pose = obs(obs(:,1) == (pose_idx - 1), :);  % match 0-based index

        for j = 1:size(obs_in_pose, 1)
            u = obs_in_pose(j, 2);
            v = obs_in_pose(j, 3);
            if abs(u) > 50 && abs(v) > 50
                idx = index_vec(landmark_id + 1);
                A1 = u * P(3,:) - P(1,:);
                A2 = v * P(3,:) - P(2,:);
                if isempty(D{landmark_id + 1})
                    D{landmark_id + 1} = [A1; A2];
                else
                    D{landmark_id + 1} = [D{landmark_id + 1}; A1; A2];
                end
                index_vec(landmark_id + 1) += 1;
            end
        end
    end
end

% Solve via SVD
landmarks = struct('id', {}, 'pos', {}, 'obs', {});
ids = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
iter = 0;

for i = 1:length(selected_ids)
    landmark_id = selected_ids(i);
    A = D{landmark_id + 1};

    if isempty(A) || size(A,1) < 4
        fprintf('LM %d skipped: not enough observations\n', landmark_id);
        continue;
    end

    [~, ~, V] = svd(A);
    X_hom = V(:,end);
    if abs(X_hom(4)) < 1e-6
        fprintf('LM %d skipped: bad homogeneous scale\n', landmark_id);
        continue;
    end
    X = X_hom(1:3) / X_hom(4);
    if abs(X(1)) > 12 || abs(X(2)) > 12 || abs(X(3)) > 3
        fprintf('LM %d skipped: out of bounds X = [%.2f %.2f %.2f]\n', landmark_id, X);
        continue;
    end

    iter += 1;
    landmarks(iter).id = landmark_id;
    landmarks(iter).pos = X;
    landmarks(iter).obs = size(A,1) / 2;
    ids(landmark_id) = iter;
end

fprintf('[TRIANGULATE_SIMPLE] Done. Triangulated %d landmarks.\n', iter);
end
