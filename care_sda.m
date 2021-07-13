function X = care_sda(A, B, H, R)
state_dimension = length(A);

r = 2.4; %SDA's author suggested the value between 2.1~2.6
I = eye(state_dimension);
G = B*inv(R)*transpose(B);
A_trans = transpose(A);
A_r = A - (r*I);

iteration_times = 0;

%solve CARE with SDA
A_hat_last = I + 2*r*inv(A_r + G*inv(transpose(A_r))*H);
G_hat_last = 2*r*inv(A_r)*G*inv(transpose(A_r) + H*inv(A_r)*G);
H_hat_last = 2*r*inv(transpose(A_r) + H*inv(A_r)*G)*H*inv(A_r);

while 1
    iteration_times = iteration_times + 1;
    
    %reduce redundent calculation by pre-calculating repeated terms
    inv_I_plus_H_G = inv(I + (H_hat_last * G_hat_last));
    transpose_A_hat_last = transpose(A_hat_last);
    
    %update
    A_hat_new = A_hat_last * inv(I + G_hat_last * H_hat_last) * A_hat_last;
    G_hat_new = G_hat_last + (A_hat_last * G_hat_last * inv_I_plus_H_G * transpose_A_hat_last);
    H_Hat_new = H_hat_last + (transpose_A_hat_last * inv_I_plus_H_G * H_hat_last * A_hat_last);
    
    %matrix norms
    norm_H_last = norm(H_hat_last);
    norm_H_now = norm(H_Hat_new);
    
    %prepare next iteration
    A_hat_last = A_hat_new;
    G_hat_last = G_hat_new;
    H_hat_last = H_Hat_new;
    
    %stop iteration if converged
    if abs(norm_H_now - norm_H_last) < 0.000000001 %* abs(norm_H_now)
        break;
    end
    
    %disp(iteration_times);
end

X = H_Hat_new;
end