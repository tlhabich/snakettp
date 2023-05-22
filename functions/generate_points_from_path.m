function points = generate_points_from_path(spline_struct, n, lengths, delta)
    % generate n segments from spline_struct
    
    % spline_struct     : [matlab spline structure] native matlab spline structure
    % n                 : [1,1] number of joints
    % lengths           : [1,n] length of each link of the robot
    % delta             : [1,1] increment for arc length calculation (numerical integration)
    
    t = delta;
    i = 1;
    arc_length = 0;
    points = zeros(n,3);
    while i < n + 1
        xyz1 = fnval(spline_struct,t-delta);
        xyz2 = fnval(spline_struct,t);
        arc_length = arc_length + norm(xyz2- xyz1);
        if arc_length >= lengths(i)
            points(i,:) = xyz2';
            i = i + 1;
            arc_length = 0;
        end
        t = t + delta;
    end
end

