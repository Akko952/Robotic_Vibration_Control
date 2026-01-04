function A = vector_to_skew_symmetric(v)
    % vector_to_skew_symmetric: Convert a 3D vector into a skew-symmetric matrix
    %
    % Input:
    %   v - a 3D vector [x, y, z]
    %
    % Output:
    %   A - a 3x3 skew-symmetric matrix

    % Check if the input is a 3D vector
    if length(v) ~= 3
        error('Input must be a 3D vector');
    end

    % Extract components of the vector
    x = v(1);
    y = v(2);
    z = v(3);

    % Construct the 3x3 skew-symmetric matrix
    A = [  0  -z   y;
           z   0  -x;
          -y   x   0 ];
end