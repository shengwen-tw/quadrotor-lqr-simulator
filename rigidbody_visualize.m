%plot_size: size of the 3d space to plot, i.e., [size_x; size_y; size_z]
%rigidbody_pos: array of rigidbody's position went before
%rigidbody_R: array of rigidbody's attitude (DCM) tilted before
%iteration_times: total count of the rigidbodys pose
%sleep_time: the time to sleep after every iteration of visualization
%skip_cnt: skip count of the visualization since it is too slow
function rigidbody_visualize(plot_size, rigidbody_pos, rigidbody_R, iterate_times, sleep_time, skip_cnt)
%define shape of quadrotor
[x1 y1 z1] = cylinder([0.2 0.2]);
[x2, y2, z2] = cylinder([0.15, 0.0]);

%set initial state of the rigidbody
i11 =  z1; i12 = y1; i13 = x1;
i21 = -z1; i22 = x1; i23 = y1;
i31 = y1; i32 = z1; i33 = x1;
i41 = y1; i42 =-z1; i43 = x1;
i51 = x2 + 1; i52 = y2; i53 = z2;
i61 = x2 - 1; i62 = y2; i63 = z2;
i71 = x2; i72 = y2 + 1; i73 = z2;
i81 = x2; i82 = y2 - 1; i83 = z2;

p11 = i11; p12 = i12; p13 = i13;
p21 = i21; p22 = i22; p23 = i23;
p31 = i31; p32 = i32; p33 = i33;
p41 = i41; p42 = i42; p43 = i43;
p51 = i51; p52 = i52; p3 = i53;
p61 = i61; p62 = i62; p3 = i63;
p71 = i71; p72 = i72; p3 = i73;
p81 = i81; p82 = i82; p3 = i83;

figure('Name', 'simulation visualization (NED)')

disp('Start timing the elsapsed time of rigibody visualization:');
tic();
for i = 1: skip_cnt: iterate_times
    sx = plot_size(1);
    sy = plot_size(2);
    sz = plot_size(3);
    axes('XLim',[-sx sx],'YLim',[-sy sy],'ZLim',[-sz sz]);
    
    [p11, p12, p13] = cylinder_transform(i11, i12, i13, rigidbody_pos(:, i), rigidbody_R(:, :, i));
    [p21, p22, p23] = cylinder_transform(i21, i22, i23, rigidbody_pos(:, i), rigidbody_R(:, :, i));
    [p31, p32, p33] = cylinder_transform(i31, i32, i33, rigidbody_pos(:, i), rigidbody_R(:, :, i));
    [p41, p42, p43] = cylinder_transform(i41, i42, i43, rigidbody_pos(:, i), rigidbody_R(:, :, i));
    [p51, p52, p53] = cylinder_transform(i51, i52, i53, rigidbody_pos(:, i), rigidbody_R(:, :, i));
    [p61, p62, p63] = cylinder_transform(i61, i62, i63, rigidbody_pos(:, i), rigidbody_R(:, :, i));
    [p71, p72, p73] = cylinder_transform(i71, i72, i73, rigidbody_pos(:, i), rigidbody_R(:, :, i));
    [p81, p82, p83] = cylinder_transform(i81, i82, i83, rigidbody_pos(:, i), rigidbody_R(:, :, i));
    
    surface(p11, p12, p13, 'FaceColor', 'red');
    surface(p21, p22, p23, 'FaceColor', 'blue');
    surface(p31, p32, p33, 'FaceColor', 'yellow');
    surface(p41, p42, p43, 'FaceColor', 'green');
    surface(p51, p52, p53, 'FaceColor', 'red');
    surface(p61, p62, p63, 'FaceColor', 'blue');
    surface(p71, p72, p73, 'FaceColor', 'yellow');
    surface(p81, p82, p83, 'FaceColor', 'green');
    
    
    %display
    view(3)
    grid on;
    pause(sleep_time);
    
    if (i + skip_cnt < iterate_times)
        clf;
    end
end
toc();
end

function [ret_px, ret_py ret_pz] = cylinder_transform(px, py, pz, pos, R)
ring1_old = [px(1,:)', py(1,:)', pz(1,:)'];
ring2_old = [px(2,:)', py(2,:)', pz(2,:)'];
ring1_rotated = ring1_old * R;
ring2_rotated = ring2_old * R;

rotated_x = [ring1_rotated(:,1), ring2_rotated(:,1)];
rotated_y = [ring1_rotated(:,2), ring2_rotated(:,2)];
rotated_z = [ring1_rotated(:,3), ring2_rotated(:,3)];

ret_px = rotated_x + pos(1);
ret_py = rotated_y + pos(2);
ret_pz = rotated_z - pos(3); %input z is in NED frame, convert to sea level height
end
