%Numerical geometric Jacobian of snake robots 
%Input:
%q double [n+1 x 1]
%   current joint angles (including feeder)
%frame_nr double
%   Number of desired frame
%link_length double
%   Length of all links (actuator height)
%Output:
%J_frame_nr double [6 x frame_nr]
%   Geometric Jacobian to Actuator #frame_nr

function J_frame_nr = jgeom_num(q,frame_nr, link_length)
%#codegen
%$cgargs {zeros(31,1),zeros(1,1),zeros(1,1)}
    J_frame_nr=zeros(6,frame_nr);
    for i = 1:frame_nr
        T_last=fkine_num(q,i-1,link_length);
        e_z=T_last(1:3,1:3)*[0;0;1];
        r_frame_nr=fkine_num(q,frame_nr,link_length)*[0;0;0;1]-T_last*[0;0;0;1];
        if i == 1 %Linear Feeder
            J_frame_nr(:,i)=[e_z;0;0;0];
        else
            J_frame_nr(:,i)=[cross(e_z,r_frame_nr(1:3));e_z];
        end
    end
end