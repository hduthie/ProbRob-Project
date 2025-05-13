source("utils_final/geometry_helpers_3d.m");
source("utils_final/total_least_squares_indices.m");
source("utils_final/total_least_squares_landmarks.m");
source("utils_final/total_least_squares_poses.m");
source("utils_final/total_least_squares_projections.m");

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


function [XR, XL, chi_stats_p, num_inliers_p, chi_stats_r, num_inliers_r, H, b, iteration] = ...
    doTotalLS(XR, XL, Zp, projection_associations, Zr, pose_associations, ...
              num_poses, num_landmarks, num_iterations, damping, ...
              kernel_threshold_proj, kernel_threshold_pose)

  global pose_dim;
  global landmark_dim;

  system_size = pose_dim * num_poses + landmark_dim * num_landmarks;

  chi_stats_p = zeros(num_iterations, 1);
  num_inliers_p = zeros(num_iterations, 1);
  chi_stats_r = zeros(num_iterations, 1);
  num_inliers_r = zeros(num_iterations, 1);

  iteration = 1;
  error = 1e6;

  while iteration <= num_iterations && error > 1e-6
    printf("Iteration %d\n", iteration);

    % Projections
    [H_proj, b_proj, chi_p, inliers_p] = linearizeProjections(XR, XL, Zp, ...
        projection_associations, num_poses, num_landmarks, kernel_threshold_proj);
    chi_stats_p(iteration) = chi_p;
    num_inliers_p(iteration) = inliers_p;

    % Poses
    [H_pose, b_pose, chi_r, inliers_r] = linearizePoses(XR, Zr, ...
        pose_associations, num_poses, num_landmarks, kernel_threshold_pose);
    chi_stats_r(iteration) = chi_r;
    num_inliers_r(iteration) = inliers_r;

    % Construct system
    H = H_proj + H_pose;
    b = b_proj + b_pose;

    % Add damping
    H += eye(system_size) * damping;

    % Fix the first pose
    dx = zeros(system_size, 1);
    dx(pose_dim+1:end) = -H(pose_dim+1:end, pose_dim+1:end) \ b(pose_dim+1:end);

    % Apply update
    [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx);

    error = sum(abs(dx));
    printf("Error: %.6f\n", error);
    iteration += 1;
  endwhile

  iteration -= 1;  % account for last increment
endfunction

