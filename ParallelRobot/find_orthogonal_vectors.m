function [omega_si2, omega_si3] = find_orthogonal_vectors(nu_pi)
    % find_orthogonal_vectors: Calculate two orthogonal unit vectors
    % that are perpendicular to the input vector.
    %
    % Input:
    %   nu_pi - A non-zero 3D vector
    %
    % Output:
    %   omega_si2 - A unit vector orthogonal to nu_pi
    %   omega_si3 - Another unit vector orthogonal to both nu_pi and omega_si2

    % Normalize the input vector to ensure proper calculations
    nu_pi = nu_pi / norm(nu_pi);

    % Check if nu_pi is parallel to [1; 0; 0], avoid numerical issues
    temp = cross(nu_pi, [1; 0; 0]);
    if norm(temp) < 1e-6
        % If nu_pi is parallel to [1; 0; 0], use [0; 1; 0] instead
        temp = cross(nu_pi, [0; 1; 0]);
    end

    % Calculate the first orthogonal unit vector omega_si2
    omega_si2 = temp / norm(temp);

    % Calculate the second orthogonal unit vector omega_si3
    omega_si3 = cross(nu_pi, omega_si2);

    % Ensure the output vectors are normalized
    omega_si3 = omega_si3 / norm(omega_si3);
end