function visualize_reprojection(XR, XL, Zp, projection_associations, landmark_index, pose_index)
  global K image_rows image_cols;

  clf; hold on; axis equal;
  title(sprintf('Landmark %d reprojection from Pose %d', landmark_index, pose_index));

  % Extract 3D landmark and camera pose
  Xl = XL(:, landmark_index);
  Xr = XR(:, :, pose_index);

  % Project the 3D point
  z_proj = projectPoint(Xr, Xl);

  % Find the measured 2D point
  idxs = find(projection_associations(1,:) == pose_index & projection_associations(2,:) == landmark_index);
  if isempty(idxs)
    disp('No matching measurement found.');
    return;
  endif

  z_measured = Zp(:, idxs(1));  % use first match if multiple

  % Plot the image plane
  plot([0 image_cols image_cols 0 0], [0 0 image_rows image_rows 0], 'k--');

  % Plot measured vs projected
  plot(z_measured(1), z_measured(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);  % ground truth
  plot(z_proj(1), z_proj(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);         % projection

  legend('Image frame', 'Measured (green)', 'Projected (red)');
  xlabel('u'); ylabel('v');
  xlim([0 image_cols]); ylim([0 image_rows]);
  set(gca, 'YDir','reverse');  % flip for image coordinates
endfunction
