function data = load_data()
% LOAD_DATA Loads the dataset files for planar monocular SLAM
% Returns:
%   data.camera       - struct with intrinsics, transform, etc.
%   data.trajectory   - matrix [pose_id, odom(1:3), gt(1:3)]
%   data.measurements - cell array: one per frame, each with [pt_id, true_id, u, v]
%   data.world        - matrix [id, x, y, z]

base_path = 'data/';  % Adjust this to your dataset path

fprintf('[LOAD] Reading camera.dat using helper...\n');
[camera_K, cam_T, z_near, z_far, img_w, img_h] = load_camera('data/camera.dat');

data.camera.K = camera_K;
data.camera.cam_transform = cam_T;
data.camera.z_near = z_near;
data.camera.z_far = z_far;
data.camera.width = img_w;
data.camera.height = img_h;

fprintf('[INFO] Camera intrinsics:\n'); disp(camera_K);
fprintf('[INFO] Camera to robot transform:\n'); disp(cam_T);
fprintf('[INFO] z_near=%.2f, z_far=%.2f, width=%d, height=%d\n', ...
    z_near, z_far, img_w, img_h);


fprintf('[LOAD] Loading trajectory data...\n');
traj_file = fullfile(base_path, 'trajectory.dat');
trajectory = load(traj_file);  % [pose_id, odom(x,y,theta), gt(x,y,theta)]
data.trajectory = trajectory;

fprintf('[LOAD] Loaded %d trajectory entries.\n', size(trajectory, 1));

% fprintf('[PLOT] Plotting odometry vs ground truth...\n');
% figure; hold on; grid on;
% plot(trajectory(:,2), trajectory(:,3), 'b-', 'DisplayName', 'Odometry');
% plot(trajectory(:,5), trajectory(:,6), 'g--', 'DisplayName', 'Ground Truth');
% xlabel('X'); ylabel('Y'); title('Trajectory: Odometry vs GT');
% legend; axis equal;

fprintf('[LOAD] Loading world ground truth landmarks...\n');
world_file = fullfile(base_path, 'world.dat');
world = load(world_file);  % [landmark_id, x, y, z]
data.world = world;

% fprintf('[PLOT] Plotting known world landmarks...\n');
% figure; scatter3(world(:,2), world(:,3), world(:,4), 20, 'filled');
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('Ground Truth Landmarks (world.dat)');
% grid on; axis equal;

fprintf('[LOAD] Parsing measurement files...\n');
data.measurements = parse_measurements(base_path);
fprintf('[DONE] Measurement parsing complete. %d unique landmarks observed.\n', ...
    data.measurements.Count);

num_frames = data.measurements.Count;
fprintf('[DONE] Data loading complete. %d frames of measurements loaded.\n', num_frames);
% fprintf('[DEBUG] Press any key to continue after reviewing plots...\n');
% pause;  
end
