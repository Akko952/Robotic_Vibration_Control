function omega = find_orthogonal_vectors_inition(v1, v2)
    % find_common_orthogonal_vector
    % Calculate a unit vector orthogonal to both input vectors.
    %
    % Input:
    %   v1, v2 - two 3D non-zero vectors
    %
    % Output:
    %   omega  - a unit vector orthogonal to both v1 and v2

    v1 = v1 / norm(v1);
    v2 = v2 / norm(v2);

    omega = cross(v1, v2);

    if norm(omega) < 1e-6
        error('Input vectors are parallel or nearly parallel; orthogonal vector is not unique.');
    end

    omega = omega / norm(omega);
end