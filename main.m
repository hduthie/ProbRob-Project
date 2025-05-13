% % =========================================================================
% % MAIN.M - Planar Monocular SLAM
% % =========================================================================
% % This script loads the dataset, triangulates 3D landmarks using noisy
% % odometry and 2D image measurements, and prepares for bundle adjustment.
% % =========================================================================
% addpath('utils');  % Add utils folder to path for helper functions
% source "utils/total_least_squares.m"
% clear; clc; close all;

% %% --- Load dataset -------------------------------------------------------
% fprintf('[INFO] Loading dataset...\n');
% [data] = load_data();  % You'll implement this next

% % data.camera          = struct with intrinsics and extrinsics
% % data.trajectory      = [N x 7] matrix: [pose_id, odom(1:3), gt(1:3)]
% % data.measurements    = cell array where each cell = [pt_id, true_id, u, v]
% % data.world           = [N_landmarks x 4]: [id, x, y, z]

% %% --- Triangulate initial 3D landmarks -----------------------------------
% % fprintf('[INFO] Triangulating initial 3D points...\n');
% % [landmarks_init] = triangulate_all(data);
% % fprintf('[PLOT] Visualizing triangulated landmarks vs ground truth...\n');
% % visualize_landmarks(landmarks_init, data);

% % --- Filter triangulated landmarks using GT ---
% % max_error = 0.5;  % adjust as needed
% % landmarks_init = filter_landmarks(landmarks_init, data.world, max_error);

% % landmarks_init = filter_by_reprojection_error(landmarks_init, data, 100);


% %% --- Evaluate triangulated map -----------------------------------------
% % fprintf('[INFO] Evaluating initial map...\n');
% % rmse_map = evaluate_map(landmarks_init, data.world);
% % fprintf('[INFO] RMSE of initial map: %.4f\n', rmse_map);


% % Select and triangulate only first 5 landmark IDs
% landmark_ids = sort(cell2mat(keys(data.measurements)));
% selected_ids = landmark_ids(1:5);
% [landmarks_init, id_map] = triangulate_simple(data);
% % fprintf('[VISUAL] Plotting selected landmarks, GT, and observing poses...\n');
% fprintf('[PLOT] Visualizing triangulated landmarks vs ground truth...\n');
% visualize_landmarks(landmarks_init, data);

% % % Prepare figure
% % figure; hold on; grid on;
% % title('Triangulated Landmarks, GT Positions, and Observing Poses');
% % xlabel('X'); ylabel('Y'); zlabel('Z');
% % axis equal;

% % % --- Plot all robot poses (trajectory) ---
% % plot3(data.trajectory(:,2), data.trajectory(:,3), zeros(size(data.trajectory,1),1), 'k--', 'DisplayName', 'Odometry');

% % % --- Plot poses that observed each landmark ---
% % colors = lines(length(landmarks_init));
% % for i = 1:length(landmarks_init)
% %     lm = landmarks_init(i);
% %     if ~isKey(data.measurements, lm.id), continue; end

% %     % Observing pose indices
% %     obs = data.measurements(lm.id);  % [pose_idx, u, v]
% %     pose_ids = unique(obs(:,1)) + 1;  % 1-based

% %     % Plot each observing pose
% %     pose_xyz = data.trajectory(pose_ids, 2:4);  % X, Y, theta
% %     scatter3(pose_xyz(:,1), pose_xyz(:,2), zeros(size(pose_ids)), 40, colors(i,:), 'filled', ...
% %         'DisplayName', sprintf('Poses for LM %d', lm.id));
% % end

% % % --- Plot triangulated landmark positions ---
% % for i = 1:length(landmarks_init)
% %     plot3(landmarks_init(i).pos(1), landmarks_init(i).pos(2), landmarks_init(i).pos(3), ...
% %         'o', 'MarkerEdgeColor', colors(i,:), 'MarkerFaceColor', colors(i,:), 'DisplayName', sprintf('LM %d (est)', landmarks_init(i).id));
% % end

% % % --- Plot ground truth landmark positions ---
% % for i = 1:length(landmarks_init)
% %     gt_row = data.world(data.world(:,1) == landmarks_init(i).id, :);
% %     if isempty(gt_row), continue; end
% %     plot3(gt_row(2), gt_row(3), gt_row(4), 'x', 'Color', colors(i,:), 'LineWidth', 2, ...
% %         'DisplayName', sprintf('LM %d (GT)', landmarks_init(i).id));
% % end

% % legend show;
% % pause;
% % fprintf('[INFO] Triangulating initial 3D points...\n');
% % [landmarks_init2] = triangulate_simple(data);
% % fprintf('[PLOT] Visualizing triangulated landmarks vs ground truth...\n');
% % visualize_landmarks(landmarks_init2, data);


% % landmarks_init = struct array: id, position (3x1), observations, etc.
% % figure; hold on; grid on;
% % for i = 1:size(data.trajectory,1)
% %     T = se2_to_SE3(data.trajectory(i, 2:4)) * data.camera.cam_transform;
% %     cam_pos = T(1:3,4);
% %     plot3(cam_pos(1), cam_pos(2), cam_pos(3), 'bo');
% % end
% % title('Camera Positions'); xlabel('X'); ylabel('Y'); zlabel('Z'); axis equal;

% %% --- (Optional) Visualize initial map -----------------------------------
% % fprintf('[PLOT] Visualizing triangulated landmarks vs ground truth...\n');
% % visualize_landmarks(landmarks_init, data);

% %% --- Prepare for Bundle Adjustment -------------------------------------
% fprintf('[INFO] Preparing for Bundle Adjustment...\n');

% % 1. Convert odometry to SE(3) pose guesses
% num_poses = size(data.trajectory, 1);
% XR_guess = zeros(4,4,num_poses);
% for i = 1:num_poses
%     XR_guess(:,:,i) = se2_to_SE3(data.trajectory(i,2:4));  % odometry
% end

% % 2. Sort triangulated landmarks by ID
% all_ids = [landmarks_init.id];
% [~, order] = sort(all_ids);
% landmarks_sorted = landmarks_init(order);  % sorted by ID

% % 3. Convert measurements Map → per-frame array
% frame_measurements = cell(num_poses, 1);
% for lid = keys(data.measurements)
%     landmark_id = lid{1};
%     obs_list = data.measurements(landmark_id);  % [pose_idx, u, v]

%     for i = 1:size(obs_list,1)
%         pose_idx = obs_list(i,1) + 1;  % 1-based indexing
%         u = obs_list(i,2);
%         v = obs_list(i,3);
%         row = [i, landmark_id, u, v];  % [obs_id, lm_id, u, v]
%         frame_measurements{pose_idx} = [frame_measurements{pose_idx}; row];
%     end
% end

% % 4. Build associations only for landmarks actually triangulated
% id_to_index = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
% for i = 1:length(landmarks_sorted)
%     id_to_index(landmarks_sorted(i).id) = i;
% end

% Zp = [];
% projection_associations = [];
% used_index_set = [];

% for t = 1:num_poses
%     frame_obs = frame_measurements{t};  % [pt_id, lm_id, u, v]
%     for i = 1:size(frame_obs,1)
%         lm_id = frame_obs(i,2);
%         if isKey(id_to_index, lm_id)
%             pos = id_to_index(lm_id);  % index into landmarks_sorted
%             uv = frame_obs(i,3:4)';
%             Zp = [Zp, uv];
%             projection_associations = [projection_associations, [t; pos]];
%             used_index_set = [used_index_set, pos];
%         end
%     end
% end

% Zp = [];
% projection_associations = [];

% for t = 1:num_poses
%     obs = data.measurements{t};  % [pt_id, landmark_id, u, v]
%     for i = 1:size(obs,1)
%         lm_id = obs(i,2);
%         if isKey(id_map, lm_id)
%             u = obs(i,3);
%             v = obs(i,4);
%             col = id_map(lm_id);
%             Zp = [Zp, [u; v]];
%             projection_associations = [projection_associations, [t; col]];
%         end
%     end
% end


% % 5. Keep only used landmarks
% used_ids = unique(used_index_set);
% num_used_landmarks = length(used_ids);

% % 6. Create remapping from old index → new compact index
% id_map = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
% for new_idx = 1:num_used_landmarks
%     old_idx = used_ids(new_idx);
%     id_map(old_idx) = new_idx;
% end

% % 7. Filter and rebuild XL_guess
% XL_guess = zeros(3, num_used_landmarks);
% for i = 1:num_used_landmarks
%     XL_guess(:, i) = landmarks_sorted(used_ids(i)).pos;
% end

% % 8. Remap projection associations to compact indices
% for i = 1:size(projection_associations, 2)
%     old = projection_associations(2,i);
%     projection_associations(2,i) = id_map(old);
% end

% % 9. Build odometry edge constraints
% Zr = zeros(4,4,num_poses-1);
% pose_associations = zeros(2, num_poses-1);
% for i = 1:(num_poses-1)
%     T_i = XR_guess(:,:,i);
%     T_j = XR_guess(:,:,i+1);
%     Zr(:,:,i) = inv(T_i) * T_j;
%     pose_associations(:,i) = [i; i+1];
% end

% % 10. Not using Zl data
% Zl = zeros(3,0);
% landmark_associations = zeros(2,0);

% % 11. Update number of landmarks
% num_landmarks = num_used_landmarks;


% %% --- Pose Graph Optimization (Bundle Adjustment) ------------------------
% fprintf('[INFO] Running Bundle Adjustment...\n');
% % [optimized_poses, optimized_landmarks] = pose_graph_optimization(data, landmarks_init);

% damping = 0;
% kernel_threshold = 1e3;
% num_iterations = 10;

% % Convert to double for optimization
% Zp = double(Zp);
% projection_associations = double(projection_associations);
% Zr = double(Zr);
% pose_associations = double(pose_associations);

% assert(all(pose_associations(:) >= 1), 'pose_associations contains zero or negative indices!');


% [XR, XL, chi_l, inliers_l, chi_p, inliers_p, chi_r, inliers_r, H, b] = ...
%     doTotalLS(XR_guess, XL_guess, ...
%               Zl, landmark_associations, ...
%               Zp, projection_associations, ...
%               Zr, pose_associations, ...
%               num_poses, num_landmarks, ...
%               num_iterations, damping, kernel_threshold);




% %% --- Evaluate results ---------------------------------------------------
% fprintf('[INFO] Evaluating results...\n');

% % evaluate_trajectory(data.trajectory, optimized_poses);
% % evaluate_map(optimized_landmarks, data.world);

% fprintf('[DONE] SLAM pipeline finished.\n');








% =========================================================================
% MAIN.M - Planar Monocular SLAM 
% =========================================================================

addpath('utils');
source "utils/total_least_squares.m"
clear; clc; close all;

%% --- Load dataset -------------------------------------------------------
fprintf('[INFO] Loading dataset...\n');
data = load_data();
% visualize_landmarks( data);

%% --- Triangulate initial 3D landmarks -----------------------------------
landmark_ids = sort(cell2mat(keys(data.measurements)));
selected_ids = landmark_ids;  % or use landmark_ids(1:5) for subset

[landmarks_init, id_map] = triangulate_simple(data, selected_ids);
% [landmarks_init] = triangulate_all(data);
% [landmarks_init] = triangulate_best_pair(data);
 
fprintf('[PLOT] Visualizing triangulated landmarks vs ground truth...\n');
visualize_landmarks( data, landmarks_init);


% === Evaluate triangulated landmark reprojection ===
fprintf("\n[INFO] Evaluating triangulated landmark reprojection...\n");

evaluate_map(landmarks_init, data);

% pause;
num_landmarks = length(landmarks_init);
XR_guess = zeros(4,4,size(data.trajectory,1));
for i = 1:size(data.trajectory,1)
    XR_guess(:,:,i) = se2_to_SE3(data.trajectory(i,2:4));
end

errors_per_landmark = cell(1, num_landmarks);

for lm_idx = 1:num_landmarks
    lm = landmarks_init(lm_idx);
    if ~isKey(data.measurements, lm.id), continue; end
    obs = data.measurements(lm.id);  % [pose_idx, u, v]

    for j = 1:size(obs, 1)
        pose_idx = obs(j,1) + 1;  % convert to 1-based indexing
        z_measured = obs(j,2:3)';
        Xr = XR_guess(:,:,pose_idx);
        z_proj = projectPoint(Xr, lm.pos);

        if all(z_proj > 0)
            reproj_error = norm(z_proj - z_measured);
            errors_per_landmark{lm_idx}(end+1) = reproj_error;
        end
    end
end

% === Report Results ===
% fprintf("\n[RESULTS] Triangulated Landmarks — Reprojection Error:\n");
% for i = 1:num_landmarks
%     errs = errors_per_landmark{i};
%     if ~isempty(errs)
%         fprintf("LM %3d — Mean: %6.2f px | Obs: %2d\n", i, mean(errs), length(errs));
%     else
%         fprintf("LM %3d — No valid projections\n", i);
%     end
% end
% fprintf("\n[INFO] Done evaluating triangulated landmarks.\n");

% pause;


%% --- Prepare for Bundle Adjustment -------------------------------------
fprintf('[INFO] Preparing for Bundle Adjustment...\n');

% 1. Convert odometry to SE(3) pose guesses
fprintf('[INFO] Converting odometry to SE(3) pose guesses...\n');
num_poses = size(data.trajectory, 1);
XR_guess = zeros(4,4,num_poses);
for i = 1:num_poses
    XR_guess(:,:,i) = se2_to_SE3(data.trajectory(i,2:4));
end

% 2. Build XL_guess and id_map (landmark_id -> column index)
fprintf('[INFO] Building XL_guess and id_map...\n');
XL_guess = zeros(3, length(landmarks_init));
id_map = containers.Map('KeyType', 'int32', 'ValueType', 'int32');
for i = 1:length(landmarks_init)
    XL_guess(:, i) = landmarks_init(i).pos;
    id_map(landmarks_init(i).id) = i;
end
num_landmarks = size(XL_guess, 2);

% 3. Build Zp and projection_associations
fprintf('[INFO] Building Zp and projection_associations...\n');
Zp = [];
projection_associations = [];

keys_list = keys(data.measurements);
for i = 1:length(keys_list)
    lm_id = keys_list{i};
    if ~isKey(id_map, lm_id), continue; end
    col = id_map(lm_id);
    obs = data.measurements(lm_id);  % [pose_idx, u, v]

    for j = 1:size(obs,1)
        pose_idx = obs(j,1) + 1;  % convert to 1-based
        uv = obs(j,2:3)';
        Zp = [Zp, uv];
        projection_associations = [projection_associations, [pose_idx; col]];
    end
end


% 4. Build odometry edge constraints
fprintf('[INFO] Building odometry edge constraints...\n');
Zr = zeros(4,4,num_poses-1);
pose_associations = zeros(2, num_poses-1);
for i = 1:(num_poses-1)
    T_i = XR_guess(:,:,i);
    T_j = XR_guess(:,:,i+1);
    Zr(:,:,i) = inv(T_i) * T_j;
    pose_associations(:,i) = [i; i+1];
end

% 5. No local measurements (Zl)
fprintf('[INFO] No local measurements (Zl)...\n');
Zl = zeros(3,0);
landmark_associations = zeros(2,0);

% Compute reprojection error per landmark before BA
fprintf("\n[INFO] Computing reprojection error per landmark...\n");
num_landmarks = size(XL_guess, 2);
errors_per_landmark = cell(1, num_landmarks);

for k = 1:size(Zp, 2)
    pose_idx = projection_associations(1, k);
    lm_idx = projection_associations(2, k);
    Xr = XR_guess(:, :, pose_idx);
    Xl = XL_guess(:, lm_idx);
    z_measured = Zp(:, k);
    z_proj = projectPoint(Xr, Xl);
    
    if all(z_proj > 0)
        reproj_error = norm(z_proj - z_measured);
        errors_per_landmark{lm_idx}(end + 1) = reproj_error;
    end
end

% Filter by reprojection error?
Print average reprojection error for each landmark
fprintf("\nAverage reprojection error per landmark:\n");
for i = 1:num_landmarks
    errs = errors_per_landmark{i};
    if ~isempty(errs)
        avg_err = mean(errs);
        fprintf("Landmark %d: %.2f pixels (from %d obs)\n", i, avg_err, length(errs));
    else
        fprintf("Landmark %d: no valid projections\n", i);
    end
end
pause;



%% --- Bundle Adjustment --------------------------------------------------
fprintf('[INFO] Running Bundle Adjustment...\n');
damping = 1e-4;
kernel_threshold_proj = 5000;
kernel_threshold_pose = 0.01;
kernel_threshold = 0.01;
num_iterations = 5;

% Zp = double(Zp);
% projection_associations = double(projection_associations);
% Zr = double(Zr);
% pose_associations = double(pose_associations);

assert(all(pose_associations(:) >= 1), 'pose_associations contains zero or negative indices');

plotting = false ;
[XR, XL, chi_stats_l, num_inliers_l, ...
          chi_stats_p, num_inliers_p, ...
          chi_stats_r, num_inliers_r, ...
          H, b] = doTotalLS(XR_guess, XL_guess, ...
                            Zl, landmark_associations, ...
                            Zp, projection_associations, ...
                            Zr, pose_associations, ...
                            num_poses, ...
                            num_landmarks, ...
                            num_iterations, ...
                            damping, ...
                            kernel_threshold, ...
                            plotting,...
                            data);

 

% --- Display Full Results --------------------------------------------------
% === Recompute estimated trajectory from XR ===
traj_estimated = zeros(3, num_poses);
for i = 1:num_poses
  traj_estimated(:, i) = t2v(XR(:,:,i))(1:3);  % [x; y; theta]
end

% Extract odometry and GT trajectory
traj_guess = data.trajectory(:, 2:4)';  % [x; y; theta], size 3xN
traj_true = data.trajectory(:, 5:7)';  % [x; y; theta], size 3xN

% Extract GT landmark positions
XL_true = data.world(:, 2:4)';  % [x; y; z], size 3xN

% Extract triangulated landmarks from struct array
XL_triang = zeros(3, length(landmarks_init));
for i = 1:length(landmarks_init)
    XL_triang(:, i) = landmarks_init(i).pos;
end


% === Recompute estimated trajectory ===
traj_estimated = zeros(3, num_poses);
for i = 1:num_poses
  traj_estimated(:, i) = t2v(XR(:,:,i))(1:3);
end

% === Landmark and Pose Visualization ===
figure(1); clf;
set(gcf, 'position', [100, 100, 1600, 900]);
% sgtitle('Landmark and Poses');

% Top-left: Ground truth vs triangulated
subplot(2,2,1);
plot3(XL_true(1,:), XL_true(2,:), XL_true(3,:), 'bo', 'MarkerSize', 3); hold on;
plot3(XL_triang(1,:), XL_triang(2,:), XL_triang(3,:), 'rx', 'MarkerSize', 3);
title('Landmark ground truth and triangulation');
xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;

% Top-right: Ground truth vs optimized
subplot(2,2,2);
plot3(XL_true(1,:), XL_true(2,:), XL_true(3,:), 'bo', 'MarkerSize', 3); hold on;
plot3(XL(1,:), XL(2,:), XL(3,:), 'rx', 'MarkerSize', 3);
title('Landmark ground truth and optimization');
xlabel('X'); ylabel('Y'); zlabel('Z'); grid on;

% Bottom-left: Trajectory ground truth vs odometry
subplot(2,2,3);
plot(traj_true(1,:), traj_true(2,:), 'bo', 'MarkerSize', 3); hold on;
plot(traj_guess(1,:), traj_guess(2,:), 'rx', 'MarkerSize', 3);
axis equal; grid on;
title('Trajectory ground truth and odometry');

% Bottom-right: Trajectory ground truth vs optimized
subplot(2,2,4);
plot(traj_true(1,:), traj_true(2,:), 'bo', 'MarkerSize', 3); hold on;
plot(traj_estimated(1,:), traj_estimated(2,:), 'rx', 'MarkerSize', 3);
axis equal; grid on;
title('Trajectory ground truth and optimization');
fprintf('[DEBUG] Press any key to continue...\n');
pause;
