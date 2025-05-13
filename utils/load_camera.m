function [K, cam_transform, z_near, z_far, image_width, image_height] = load_camera(filepath)
    fid = fopen(filepath, 'r');
    if fid == -1
        error('Could not open %s', filepath);
    endif

    % Read camera intrinsics
    fgetl(fid);  % Skip 'camera matrix:' label
    K = fscanf(fid, '%f', [3, 3])';
    fgetl(fid);  % Skip empty line
    fgetl(fid);  % Skip 'cam_transform:' label

    % Read cam_transform matrix (4x4)
    cam_transform = zeros(4, 4);
    for i = 1:4
        line = fgetl(fid);
        nums = sscanf(line, '%f');
        if numel(nums) ~= 4
            error("Line %d of cam_transform does not have 4 values: '%s'", i, line);
        endif
        cam_transform(i, :) = nums';
    end

    % Read scalar values
    labels = {'z_near:', 'z_far:', 'width:', 'height:'};
    values = zeros(1, length(labels));

    for i = 1:length(labels)
        line = fgetl(fid);
        parts = strsplit(strtrim(line));
        values(i) = str2double(parts{2});
    end

    fclose(fid);

    z_near = values(1);
    z_far = values(2);
    image_width = values(3);
    image_height = values(4);
end