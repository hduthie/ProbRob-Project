function landmarks_out = filter_landmarks(landmarks_in, world_gt, max_error)
% NOT CURRENTLY USED
% FILTER_LANDMARKS filters triangulated landmarks based on ground truth proximity
%
% Inputs:
%   landmarks_in - struct array with fields 'id' and 'pos' (3×1)
%   world_gt     - matrix [id, x, y, z] from world.dat
%   max_error    - max Euclidean distance to keep a landmark
%
% Output:
%   landmarks_out - filtered subset of landmarks_in

    fprintf('[FILTER] Limiting landmarks using ground truth positions...\n');

    % Build GT lookup table
    gt_map = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    for i = 1:size(world_gt, 1)
        gt_map(world_gt(i,1)) = world_gt(i,2:4)';
    end

    filtered = [];
    for i = 1:length(landmarks_in)
        id = landmarks_in(i).id;
        if isKey(gt_map, id)
            gt_pos = gt_map(id);
            est_pos = landmarks_in(i).pos;
            err = norm(est_pos - gt_pos);
            if err <= max_error
                filtered = [filtered, i];
            end
        end
    end

    landmarks_out = landmarks_in(filtered);

    fprintf('[FILTER] Kept %d / %d landmarks (≤ %.2f units)\n', ...
        length(filtered), length(landmarks_in), max_error);
end
