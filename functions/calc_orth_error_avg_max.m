function [avg_deviation, max_deviation,points_current, orth_points] = calc_orth_error_avg_max(spline_struct, q, link_length, n, fkine)
    % calculate average and max deviation between spline and current joint
    % positions 
    % (deviation = orthogonal distance between joint i and spline)

    % spline_struct : spline structure
    % q             : [n,1] joint configuration
    % link_length   : [1,1] length of one link
    % n             : [1,1] number of joints
    % fkine         : [function handle] fkine handle with input (q, joint_index, link_length)
    
    orth_points = zeros(n, 3);
    points_current = zeros(n,3);
    deviation=zeros(n,1);
    inc=link_length/5;
    ds=0;
    for i_act = 1:n %search for every joint position i one corresponding orth_point
        T_i_act = fkine(q, i_act, link_length);
        points_current(i_act,:) = T_i_act(1:3,4);
        %get longitudinal direction
        if i_act==1
            long_dir=T_i_act(1:3,2);
        else
            long_dir=T_i_act(1:3,1);
        end
        %iterate along spline
        distance=points_current(i_act,:)'-fnval(spline_struct,ds);
%         while round(dot(long_dir,distance),3)~=0 %orthogonal distance
%             ds=ds+inc;
%             distance=points_current(i_act,:)'-fnval(spline_struct,ds);
%         end
        deviation(i_act)=norm(distance);
        orth_points(i_act,:)=fnval(spline_struct,ds);
    end

    % calculate avg deviation
    avg_deviation = sum(deviation) / n;
    % calculate max deviation
    max_deviation = max(deviation);
end