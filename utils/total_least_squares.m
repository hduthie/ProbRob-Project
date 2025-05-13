source("utils/geometry_helpers_3d.m");
source("utils/total_least_squares_indices.m");
source("utils/total_least_squares_landmarks.m");
source("utils/total_least_squares_poses.m");
source("utils/total_least_squares_projections.m");

# implementation of the boxplus
# applies a perturbation to a set of landmarks and robot poses
# input:
#   XR: the robot poses (4x4xnum_poses: array of homogeneous matrices)
#   XL: the landmark pose (3xnum_landmarks matrix of landmarks)
#   num_poses: number of poses in XR (added for consistency)
#   num_landmarks: number of landmarks in XL (added for consistency)
#   dx: the perturbation vector of appropriate dimensions
#       the poses come first, then the landmarks
# output:
#   XR: the robot poses obtained by applying the perturbation
#   XL: the landmarks obtained by applying the perturbation

function [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx)
  global pose_dim;
  global landmark_dim;

  for pose_index = 1:num_poses
    pose_matrix_index = poseMatrixIndex(pose_index, num_poses, num_landmarks);
    dxr = dx(pose_matrix_index:pose_matrix_index + pose_dim - 1);
    XR(:,:,pose_index) = v2t(dxr) * XR(:,:,pose_index);
  end

  for landmark_index = 1:num_landmarks
    landmark_matrix_index = landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);
    dxl = dx(landmark_matrix_index:landmark_matrix_index + landmark_dim - 1);
    XL(:, landmark_index) += dxl;
  end
endfunction



function [XR, XL, chi_stats_l, num_inliers_l, ...
          chi_stats_p, num_inliers_p, ...
          chi_stats_r, num_inliers_r, ...
          H, b] = doTotalLS(XR, XL, ...
                            Zl, landmark_associations, ...
                            Zp, projection_associations, ...
                            Zr, pose_associations, ...
                            num_poses, ...
                            num_landmarks, ...
                            num_iterations, ...
                            damping, ...
                            kernel_threshold)

  global pose_dim;
  global landmark_dim;
  global data;

  chi_stats_l = zeros(1, num_iterations);
  num_inliers_l = zeros(1, num_iterations);
  chi_stats_p = zeros(1, num_iterations);
  num_inliers_p = zeros(1, num_iterations);
  chi_stats_r = zeros(1, num_iterations);
  num_inliers_r = zeros(1, num_iterations);
  dx_norm = zeros(1, num_iterations);

  system_size = pose_dim * num_poses + landmark_dim * num_landmarks;
  stop_threshold = 1e-6;

  for iteration = 1:num_iterations
    fprintf('[BA] Iteration %d\n', iteration);
    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);

    if num_landmarks > 0
      [H_landmarks, b_landmarks, chi_l, inliers_l] = ...
        linearizeLandmarks(XR, XL, Zl, landmark_associations, ...
                           num_poses, num_landmarks, kernel_threshold);
      chi_stats_l(iteration) = chi_l;
      num_inliers_l(iteration) = inliers_l;

      [H_proj, b_proj, chi_p, inliers_p] = ...
        linearizeProjections(XR, XL, Zp, projection_associations, ...
                             num_poses, num_landmarks, kernel_threshold);
      chi_stats_p(iteration) = chi_p;
      num_inliers_p(iteration) = inliers_p;

      H += H_landmarks + H_proj;
      b += b_landmarks + b_proj;
    end

    [H_poses, b_poses, chi_r, inliers_r] = ...
      linearizePoses(XR, XL, Zr, pose_associations, ...
                     num_poses, num_landmarks, kernel_threshold);
    chi_stats_r(iteration) = chi_r;
    num_inliers_r(iteration) = inliers_r;

    H += H_poses;
    b += b_poses;

    H += eye(system_size) * damping;
    H(1:pose_dim, 1:pose_dim) += eye(pose_dim) * 1e6;
    b(1:pose_dim) = 0;

    dx = -H \ b;
    dx_norm(iteration) = norm(dx);

    [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx);

    fprintf('    ‖dx‖ = %.4e\n', dx_norm(iteration));
    total_chi = chi_stats_r(iteration) + chi_stats_p(iteration) + chi_stats_l(iteration);
    fprintf('    χ² total = %.4f   (r=%.4f, p=%.4f, l=%.4f)\n', ...
            total_chi, chi_stats_r(iteration), chi_stats_p(iteration), chi_stats_l(iteration));

    if dx_norm(iteration) < stop_threshold
      fprintf('[BA] Converged early at iteration %d\n', iteration);
      break;
    end

    % --- Plot trajectory and landmarks every 3 iterations ---
    if mod(iteration, 3) == 0 || iteration == num_iterations
      figure('Name', sprintf('SLAM View - Iteration %d', iteration), 'NumberTitle', 'off');
      hold on; grid on; axis equal;
      title(sprintf('Estimated vs Ground Truth - Iteration %d', iteration));
      xlabel('X'); ylabel('Y'); zlabel('Z');

      % Plot GT landmarks
      scatter3(data.world(:,2), data.world(:,3), data.world(:,4), 20, 'g', 'filled', 'DisplayName', 'GT Landmarks');

      % Plot GT and odometry trajectories
      plot3(data.trajectory(:,5), data.trajectory(:,6), zeros(num_poses,1), 'k-', 'DisplayName', 'GT Trajectory');
      plot3(data.trajectory(:,2), data.trajectory(:,3), zeros(num_poses,1), 'b--', 'DisplayName', 'Odometry');

      % Plot estimated camera trajectory
      cam_pos = zeros(num_poses,3);
      for i = 1:num_poses
        cam_pos(i,:) = XR(1:3,4,i)';
      end
      plot3(cam_pos(:,1), cam_pos(:,2), cam_pos(:,3), 'r-', 'DisplayName', 'Estimated Trajectory');

      % Plot estimated landmarks
      scatter3(XL(1,:), XL(2,:), XL(3,:), 20, 'r', 'DisplayName', 'Estimated Landmarks');
      legend;
    end
  end

  % --- Final chi squared plots ---
  figure('Name', 'BA Optimization Stats', 'NumberTitle', 'off');
  set(gcf, 'Position', [100, 100, 1000, 400]);

  subplot(1,2,1); hold on; grid on;
  plot(chi_stats_l + chi_stats_p + chi_stats_r, 'b-', 'LineWidth', 2, 'DisplayName', 'Total χ²');
  plot(chi_stats_r, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Pose χ²');
  plot(chi_stats_p, 'g:', 'LineWidth', 1.5, 'DisplayName', 'Proj. χ²');
  title('χ² over Iterations'); xlabel('Iteration'); ylabel('χ²'); legend('show');

  subplot(1,2,2); hold on; grid on;
  plot(dx_norm, 'm-', 'LineWidth', 2);
  title('Step Norm ‖dx‖ over Iterations'); xlabel('Iteration'); ylabel('‖dx‖');
endfunction

% function [XR, XL, chi_stats_p, num_inliers_p, chi_stats_r, num_inliers_r, H, b, iteration] = ...
%     doTotalLS(XR, XL, Zp, projection_associations, Zr, pose_associations, ...
%               num_poses, num_landmarks, num_iterations, damping, ...
%               kernel_threshold_proj, kernel_threshold_pose)

%   global pose_dim;
%   global landmark_dim;

%   system_size = pose_dim * num_poses + landmark_dim * num_landmarks;

%   chi_stats_p = zeros(num_iterations, 1);
%   num_inliers_p = zeros(num_iterations, 1);
%   chi_stats_r = zeros(num_iterations, 1);
%   num_inliers_r = zeros(num_iterations, 1);

%   iteration = 1;
%   error = 1e6;

%   while iteration <= num_iterations && error > 1e-6
%     printf("Iteration %d\n", iteration);

%     % Projections
%     [H_proj, b_proj, chi_p, inliers_p] = linearizeProjections(XR, XL, Zp, ...
%         projection_associations, num_poses, num_landmarks, kernel_threshold_proj);
%     chi_stats_p(iteration) = chi_p;
%     num_inliers_p(iteration) = inliers_p;

%     % Poses
%     [H_pose, b_pose, chi_r, inliers_r] = linearizePoses(XR, Zr, ...
%         pose_associations, num_poses, num_landmarks, kernel_threshold_pose);
%     chi_stats_r(iteration) = chi_r;
%     num_inliers_r(iteration) = inliers_r;

%     % Construct system
%     H = H_proj + H_pose;
%     b = b_proj + b_pose;

%     % Add damping
%     H += eye(system_size) * damping;

%     % Fix the first pose
%     dx = zeros(system_size, 1);
%     dx(pose_dim+1:end) = -H(pose_dim+1:end, pose_dim+1:end) \ b(pose_dim+1:end);

%     % Apply update
%     [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx);

%     error = sum(abs(dx));
%     printf("Error: %.6f\n", error);
%     iteration += 1;
%   endwhile

%   iteration -= 1;  % account for last increment
% endfunction

