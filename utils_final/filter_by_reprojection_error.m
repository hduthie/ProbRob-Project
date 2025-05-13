function landmarks_out = filter_by_reprojection_error(landmarks_in, data, max_reproj_error)
% Filters landmarks whose average reprojection error exceeds a threshold
% and visualizes their reprojections and error stats.

fprintf('[FILTER] Culling landmarks with reprojection error > %.2f px...\n', max_reproj_error);

K = data.camera.K;
T_cam = data.camera.cam_transform;

valid = false(1, length(landmarks_in));
err_list = zeros(1, length(landmarks_in));

for i = 1:length(landmarks_in)
    lm = landmarks_in(i);
    if ~isKey(data.measurements, lm.id), continue; end

    obs = data.measurements(lm.id);  % [pose_idx, u, v]
    total_err = 0; count = 0;

    for j = 1:size(obs,1)
        pose_idx = obs(j,1) + 1;
        uv_meas = obs(j,2:3)';

        T_world = se2_to_SE3(data.trajectory(pose_idx,2:4));
        T_cam_world = inv(T_cam * inv(T_world));
        p_cam = T_cam_world(1:3,1:3) * lm.pos + T_cam_world(1:3,4);

        if p_cam(3) <= 0, continue; end  % behind camera

        uv_proj = K * p_cam;
        uv_proj = uv_proj(1:2) / uv_proj(3);

        err = norm(uv_proj - uv_meas);
        if isfinite(err)
            total_err += err;
            count += 1;
        end
    end

    if count > 0
        avg_err = total_err / count;
        err_list(i) = avg_err;
        if avg_err <= max_reproj_error
            valid(i) = true;
        end
    end
end

landmarks_out = landmarks_in(valid);
fprintf('[FILTER] Kept %d / %d landmarks under %.1f px reprojection error\n', sum(valid), length(landmarks_in), max_reproj_error);

% --- Visual summary: histogram of all reprojection errors ---
figure;
hist(err_list, 50);
xlabel('Average Reprojection Error (px)');
ylabel('Number of Landmarks');
title(sprintf('Reprojection Error Distribution (Threshold = %.1f px)', max_reproj_error));
grid on;

% --- Visualize reprojection of first few filtered landmarks ---
fprintf('[DEBUG] Plotting reprojections for first few landmarks...\n');

for i = 1:min(5, length(landmarks_out))
    lm = landmarks_out(i);
    obs = data.measurements(lm.id);

    figure; hold on; grid on;
    title(sprintf('LM %d | Avg Error ≈ %.1f px', lm.id, err_list(lm.id + 1)));
    xlabel('u (pixels)'); ylabel('v (pixels)');
    
    for j = 1:size(obs,1)
        pose_idx = obs(j,1) + 1;
        uv_meas = obs(j,2:3)';
        T_world = se2_to_SE3(data.trajectory(pose_idx,2:4));
        T_cam_world = inv(T_cam * inv(T_world));
        p_cam = T_cam_world(1:3,1:3) * lm.pos + T_cam_world(1:3,4);

        if p_cam(3) <= 0, continue; end

        uv_proj = K * p_cam;
        uv_proj = uv_proj(1:2) / uv_proj(3);

        plot(uv_meas(1), uv_meas(2), 'go', 'MarkerSize', 6, 'LineWidth', 1.5);
        plot(uv_proj(1), uv_proj(2), 'rx', 'MarkerSize', 6, 'LineWidth', 1.5);
    end

    legend('Measured (green)', 'Projected (red)');
end

end
