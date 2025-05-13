function landmarks = triangulate_best_pair(data)
% TRIANGULATE_ALL Triangulation using best view pair with max parallax

fprintf('[TRIANGULATE] Using only optimal view pair per landmark...\n');

K = data.camera.K;
T_cam = inv(data.camera.cam_transform);
landmarks = struct('id', {}, 'pos', {}, 'obs', {});

meas_keys = keys(data.measurements);
iter = 0;

for lid = meas_keys
    landmark_id = lid{1};
    obs = data.measurements(landmark_id);  % [pose_idx, u, v]
    if size(obs,1) < 2
        continue;
    end

    % Collect camera poses and rays
    poses = {};
    rays = {};
    centers = [];
    for i = 1:size(obs,1)
        pose_idx = obs(i,1) + 1;  % 1-based
        uv = obs(i,2:3)';
        if all(abs(uv) < 50)
            continue;
        end

        T_robot = se2_to_SE3(data.trajectory(pose_idx, 2:4));
        T = T_robot * data.camera.cam_transform;
        R = T(1:3,1:3);
        t = T(1:3,4);

        ray_cam = K \ [uv; 1];
        ray_world = R * ray_cam;
        ray_world = ray_world / norm(ray_world);

        poses{end+1} = t;
        rays{end+1} = ray_world;
        centers(:,end+1) = t;
    end

    if length(poses) < 2
        continue;
    end

    % Find best pair (max parallax)
    best_angle = 0;
    best_i = 0; best_j = 0;
    for i = 1:length(poses)
        for j = i+1:length(poses)
            vi = centers(:,i);
            vj = centers(:,j);
            angle = acos(dot(vj-vi, rays{j}) / norm(vj-vi));
            if angle > best_angle
                best_angle = angle;
                best_i = i;
                best_j = j;
            end
        end
    end

    if rad2deg(best_angle) < 10
        continue;
    end

    % Triangulate from best pair (midpoint method)
    o1 = poses{best_i}; d1 = rays{best_i};
    o2 = poses{best_j}; d2 = rays{best_j};
    A = [eye(3), -d1; eye(3), -d2];
    b = [o1; o2];
    x = A \ b;
    X = x(1:3);

    iter = iter + 1;
    landmarks(iter).id = landmark_id;
    landmarks(iter).pos = X;
    landmarks(iter).obs = 2;
end

fprintf('[TRIANGULATE] Done. Triangulated %d valid landmarks.\n', iter);

% % Optional debug plot
% figure; hold on; grid on;
% for i = 1:iter
%     plot3(landmarks(i).pos(1), landmarks(i).pos(2), landmarks(i).pos(3), 'r.');
% end
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('Triangulated Landmarks (Best Pair Only)');
% axis equal; drawnow; pause;
end