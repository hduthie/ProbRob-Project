% % =========================================================================
% % MAIN.M - Planar Monocular SLAM
% % =========================================================================
% % This script loads the dataset, triangulates 3D landmarks using noisy
% % odometry and 2D image measurements, and prepares for bundle adjustment.
% % =========================================================================

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
fprintf("\n[RESULTS] Triangulated Landmarks — Reprojection Error:\n");
for i = 1:num_landmarks
    errs = errors_per_landmark{i};
    if ~isempty(errs)
        fprintf("LM %3d — Mean: %6.2f px | Obs: %2d\n", i, mean(errs), length(errs));
    else
        fprintf("LM %3d — No valid projections\n", i);
    end
end
fprintf("\n[INFO] Done evaluating triangulated landmarks.\n");

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

% Print average reprojection error for each landmark
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



%% --- Bundle Adjustment --------------------------------------------------
fprintf('[INFO] Running Bundle Adjustment...\n');
damping = 1e-4;
kernel_threshold_proj = 5000;
kernel_threshold_pose = 0.01;
kernel_threshold = 0.01;
num_iterations = 10;

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
