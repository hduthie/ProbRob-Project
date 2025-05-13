function landmarks = triangulate_all(data)
% TRIANGULATE_ALL Robust triangulation using design matrix & parallax check
% Returns struct array: id, pos, obs

fprintf('[TRIANGULATE] Using design matrix + parallax filtering...\n');

min_views = 3;
min_parallax_deg = 5.0;
K = data.camera.K;
T_cam = inv(data.camera.cam_transform);  % Inverse cam-to-robot

num_landmarks = max(cell2mat(keys(data.measurements))) + 1;  % +1 for 0-based ID
landmarks = struct('id', {}, 'pos', {}, 'obs', {});
index_vec = zeros(1, num_landmarks);
D = cell(num_landmarks, 1);

% Pass 1: Fill design matrix blocks
for pose_idx = 1:size(data.trajectory, 1)
    T_robot = se2_to_SE3(data.trajectory(pose_idx, 2:4));
    % T_robot = se2_to_SE3(data.trajectory(pose_idx, 5:7));# ground truth trajectory
    P = K * [eye(3), zeros(3,1)] * T_cam * inv(T_robot);

    % Invert pose index offset if needed
    for lid = keys(data.measurements)
        landmark_id = lid{1};
        obs = data.measurements(landmark_id);
        obs_in_frame = obs(obs(:,1) == pose_idx - 1, :);  % 0-based match

        if isempty(obs_in_frame)
            continue;
        end

        for k = 1:size(obs_in_frame,1)
            uv = obs_in_frame(k,2:3);
            if all(abs(uv) > 50)  % field-of-view filter
                idx = index_vec(landmark_id + 1); 
                row_start = 2 * idx + 1;

                Dblock = [
                    uv(1) * P(3,:) - P(1,:);
                    uv(2) * P(3,:) - P(2,:)
                ];

                if isempty(D{landmark_id + 1})
                    D{landmark_id + 1} = Dblock;
                else
                    D{landmark_id + 1} = [D{landmark_id + 1}; Dblock];
                end

                index_vec(landmark_id + 1) = index_vec(landmark_id + 1) + 1;
            end
        end
    end
end

% Pass 2: Triangulate valid landmarks
iter = 0;
for landmark_id = 0:num_landmarks - 1
    A = D{landmark_id + 1};
    if isempty(A) || size(A,1) < 2 * min_views
        continue;
    end

    % Collect camera centers for parallax check
    observing_poses = [];
    for pose_idx = 1:size(data.trajectory, 1)
        obs = data.measurements(landmark_id);
        if any(obs(:,1) == pose_idx - 1)
            observing_poses(end+1) = pose_idx;
        end
    end

    centers = [];
    for pose_idx = observing_poses
        T = se2_to_SE3(data.trajectory(pose_idx,2:4)) * data.camera.cam_transform;
        centers(:,end+1) = T(1:3,4);
    end

    % Parallax check
    max_angle = 0;
    for i = 1:size(centers,2)
        for j = i+1:size(centers,2)
            vi = centers(:,i) - centers(:,1);
            vj = centers(:,j) - centers(:,1);
            angle = acos(dot(vi,vj) / (norm(vi)*norm(vj)));
            max_angle = max(max_angle, angle);
        end
    end

    if rad2deg(max_angle) < min_parallax_deg
        continue;
    end

    % Triangulate using SVD
    [~, ~, V] = svd(A);
    X_hom = V(:,end);
    if abs(X_hom(4)) < 1e-6
        continue;
    end
    X = X_hom(1:3) / X_hom(4);

    % Store
    iter = iter + 1;
    landmarks(iter).id = landmark_id;
    landmarks(iter).pos = X;
    landmarks(iter).obs = size(A,1) / 2;
end

fprintf('[TRIANGULATE] Done. Triangulated %d valid landmarks.\n', iter);

% Plot
figure; hold on; grid on;
for i = 1:iter
    plot3(landmarks(i).pos(1), landmarks(i).pos(2), landmarks(i).pos(3), 'r.');
end
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Triangulated Landmark Positions (Design Matrix)');
axis equal;
drawnow;
pause;

end




% % function landmarks = triangulate_all(data)
% % % TRIANGULATE_ALL Estimates initial 3D landmark positions
% % % Inputs:
% % %   data - struct from load_data with camera, trajectory, and measurements
% % % Output:
% % %   landmarks - struct array with fields:
% % %       .id  - landmark ID
% % %       .pos - estimated 3D position [x; y; z]
% % %       .obs - number of observations

% % fprintf('[TRIANGULATE] Starting triangulation of all landmarks...\n');

% % cam_K = data.camera.K;
% % cam_T_cam2robot = data.camera.cam_transform; 

% % landmark_ids = keys(data.measurements);
% % num_landmarks = length(landmark_ids);
% % landmarks = struct('id', {}, 'pos', {}, 'obs', {});

% % for idx = 1:num_landmarks
% %     landmark_id = landmark_ids{idx};
% %     obs_list = data.measurements(landmark_id);  % [pose_idx, u, v]

% %     if size(obs_list,1) < 2
% %         continue  % Need at least two observations to triangulate
% %     end

% %     A = [];  % Accumulate linear system
% %     for j = 1:size(obs_list,1)
% %         pose_idx = obs_list(j,1);
% %         u = obs_list(j,2);
% %         v = obs_list(j,3);

% %         % --- Step 1: Compute pose in SE(3) (assume Z=0, planar motion)
% %         pose = data.trajectory(pose_idx+1, 2:4);  % [x, y, theta]
% %         T_robot = se2_to_SE3(pose);
% %         T_world_to_cam = T_robot * cam_T_cam2robot;
% %         T_cam_in_world = se2_to_SE3(pose) * data.camera.cam_transform;


% %         % --- Step 2: Backproject to normalized bearing vector
% %         pixel = [u; v; 1];
% %         ray_cam = cam_K \ pixel;  % in camera coords

% %         % Direction in world frame
% %         % R = T_world_to_cam(1:3,1:3);
% %         % t = T_world_to_cam(1:3,4);
% %         % ray_world = R * ray_cam;
% %         % ray_world = ray_world / norm(ray_world);  % normalize
% %         R = T_cam_in_world(1:3, 1:3);
% %         t = T_cam_in_world(1:3, 4);
% %         ray_cam = cam_K \ [u; v; 1];
% %         ray_world = R * ray_cam;
% %         ray_world = ray_world / norm(ray_world);


% %         % Build linear system for triangulation (midpoint method)
% %         o = t;  % camera center
% %         d = ray_world;

% %         % Each line: point on line = o + λ*d
% %         % We'll build a system that minimizes distance between all rays

% %         % Append terms for midpoint triangulation:
% %         % [I -d] * [X; λ] = o
% %         A_block = [eye(3) -d];
% %         b_block = o;

% %         A = [A; A_block];
% %     end

% %     % Solve overdetermined system A * [X; λ_1; λ_2; ...] = b
% %     b = A(:,end);
% %     A_trimmed = A(:,1:3);
% %     X = A_trimmed \ b;

% %     landmarks(end+1).id = landmark_id;
% %     landmarks(end).pos = X;
% %     landmarks(end).obs = size(obs_list,1);
% % end

% % fprintf('[TRIANGULATE] Done. Triangulated %d landmarks with ≥ 2 views.\n', length(landmarks));

% % % Optional debug plot
% % figure; hold on; grid on;
% % for i = 1:length(landmarks)
% %     plot3(landmarks(i).pos(1), landmarks(i).pos(2), landmarks(i).pos(3), 'r.');
% % end
% % xlabel('X'); ylabel('Y'); zlabel('Z');
% % title('Triangulated Landmark Positions');
% % axis equal;
% % drawnow;

% % end

% % function landmarks = triangulate_all(data)
% % % TRIANGULATE_ALL_DESIGN_MATRIX - Triangulates landmarks using a global design matrix
% % % Builds a linear system for each landmark and solves via SVD.
% % % Returns struct array: id, pos, obs_count

% % fprintf('[TRIANGULATE] Using global design matrix with filtering...\n');

% % % Parameters
% % min_views = 2;
% % min_parallax_deg = 5.0;  % Optional: not enforced here
% % uv_threshold = 50;       % Discard image points near center

% % K = data.camera.K;
% % T_cam = inv(data.camera.cam_transform);  % camera-to-robot inverse
% % num_poses = size(data.trajectory, 1);
% % meas_keys = keys(data.measurements);
% % num_landmarks = max(cell2mat(meas_keys)) + 1;

% % % Initialize
% % D = cell(num_landmarks, 1);  % design matrix per landmark
% % obs_count = zeros(1, num_landmarks);

% % % Pass 1: fill design matrix blocks
% % for pose_idx = 1:num_poses
% %     T_robot = se2_to_SE3(data.trajectory(pose_idx, 2:4));
% %     P = K * [eye(3), zeros(3,1)] * T_cam * inv(T_robot);

% %     for lid = meas_keys
% %         landmark_id = lid{1};
% %         obs_list = data.measurements(landmark_id);

% %         % Filter by current pose
% %         obs_in_pose = obs_list(obs_list(:,1) == (pose_idx-1), :);  % 0-based match

% %         for j = 1:size(obs_in_pose, 1)
% %             u = obs_in_pose(j, 2);
% %             v = obs_in_pose(j, 3);
% %             if abs(u) > uv_threshold && abs(v) > uv_threshold
% %                 A1 = u * P(3,:) - P(1,:);
% %                 A2 = v * P(3,:) - P(2,:);
% %                 if isempty(D{landmark_id + 1})
% %                     D{landmark_id + 1} = [A1; A2];
% %                 else
% %                     D{landmark_id + 1} = [D{landmark_id + 1}; A1; A2];
% %                 end
% %                 obs_count(landmark_id + 1) += 1;
% %             end
% %         end
% %     end
% % end

% % % Pass 2: solve via SVD for valid landmarks
% % landmarks = struct('id', {}, 'pos', {}, 'obs', {});
% % iter = 0;

% % for landmark_id = 0:num_landmarks-1
% %     A = D{landmark_id + 1};
% %     if isempty(A) || size(A,1) < 2 * min_views
% %         continue;
% %     end

% %     [~, ~, V] = svd(A);
% %     X_hom = V(:,end);
% %     if abs(X_hom(4)) < 1e-6
% %         continue;
% %     end
% %     X = X_hom(1:3) / X_hom(4);

% %     iter += 1;
% %     landmarks(iter).id = landmark_id;
% %     landmarks(iter).pos = X;
% %     landmarks(iter).obs = size(A,1) / 2;
% % end

% % fprintf('[TRIANGULATE] Done. Triangulated %d valid landmarks (≥ %d views).\n', iter, min_views);

% % % Optional: debug plot
% % figure; hold on; grid on;
% % for i = 1:iter
% %     plot3(landmarks(i).pos(1), landmarks(i).pos(2), landmarks(i).pos(3), 'r.');
% % end
% % xlabel('X'); ylabel('Y'); zlabel('Z');
% % title('Triangulated Landmarks (Design Matrix + Filtered)');
% % axis equal;
% % drawnow;
% % end

