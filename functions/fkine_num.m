%Numerical forward kinematics of snake robots  
%(Number of rotational actuators must be even!)
%Input:
%q double [n+1 x 1]
%   current joint angles (including feeder)
%frame_nr double
%   Number of desired frame
%link_length double
%   Length of all links (actuator height)
%Output:
%T_0_frame_nr double [4 x 4]
%   Single transformation between (F)_frame_nr and (F)_0
%p_all double [3 x frame_nr]
%   Positions of all links until link #frame_nr p_all=[p_1, p_2, ..., p_frame_nr]

function [T_0_frame_nr,p_all] = fkine_num(q,frame_nr, link_length)
%#codegen
%$cgargs {zeros(31,1),zeros(1,1),zeros(1,1)}
    n_total=size(q,1);%including feeder   
    p_all=zeros(3,frame_nr);
    %Get DH Parameters  
    theta = [0;q(2)-pi/2;q(3:end)];
    d = [q(1)+link_length/2;zeros(n_total-1,1)];
    a = [0; link_length * ones(n_total-2,1); link_length/2];
    alpha = (2*rem(1:n_total,2) - 1)*(-pi/2);
    alpha(n_total)=0;
    
    %Compute T
    T_0_frame_nr=eye(4);
    for i=1:frame_nr
        A_i=[cos(theta(i)),-sin(theta(i))*cos(alpha(i)),sin(theta(i))*sin(alpha(i)),a(i)*cos(theta(i));
            sin(theta(i)),cos(theta(i))*cos(alpha(i)),-cos(theta(i))*sin(alpha(i)),a(i)*sin(theta(i));
            0,sin(alpha(i)),cos(alpha(i)),d(i);
            0,0,0,1];
        T_0_frame_nr=T_0_frame_nr*A_i;
        p_all(:,i)=T_0_frame_nr(1:3,4);
    end
end
    