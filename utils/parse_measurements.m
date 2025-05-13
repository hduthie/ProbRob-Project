function landmark_obs = parse_measurements(folder)
% PARSE_MEASUREMENTS Parses all measurement files in a folder
% Returns a map: landmark_id → [pose_idx, u, v; ...]

    files = dir(fullfile(folder, 'meas-*.dat'));
    num_files = length(files);
    landmark_obs = containers.Map('KeyType', 'int32', 'ValueType', 'any');

    for i = 0:num_files - 1
        filename = sprintf('%s/meas-%05d.dat', folder, i);
        fid = fopen(filename, 'r');
        if fid == -1
            warning('[WARN] Could not open file: %s', filename);
            continue;
        end

        while ~feof(fid)
            line = strtrim(fgetl(fid));
            if startsWith(line, 'point')
                tokens = sscanf(line(6:end), '%d %d %f %f');
                point_id    = tokens(1);  % Unused
                landmark_id = tokens(2);
                u = tokens(3);
                v = tokens(4);
                obs_entry = [i, u, v];  % pose_idx, u, v

                if isKey(landmark_obs, landmark_id)
                    landmark_obs(landmark_id) = [landmark_obs(landmark_id); obs_entry];
                else
                    landmark_obs(landmark_id) = obs_entry;
                end
            end
        endwhile

        fclose(fid);
        fprintf('[LOAD] %s parsed.\n', filename);
    end
end
