function [annot] = getComplementaryAnnot( samples, name )
%function [A] = getComplementaryAnnot( samples )
%function creates complementary annotation which means that for a given
%state {i} every flipper mode which was not used is considered as forbiden.
%this imply that for current state {i} there will be only one control
%allowed.

%% fill in the remaining fields
% - annot
% - name --- see above
% - mode_c
% - time_stamps
% - mode_change
L = length(samples);%zeros means not allowed modes
A = zeros(6, L);
mode_c = zeros(1,L);
time_stamps = zeros(1,L);
mode_change = zeros(1,L);

for i = 1:L
    A(samples{i}.action.flipper_mode+1, i) = (samples{i}.final_reward>0) + 2*(samples{i}.final_reward==0);%for the current control set the annot to zero -> allowed
    
    
    mode_c(i) = samples{i}.action.flipper_mode+1;
    time_stamps(i) = samples{i}.time.sec + samples{i}.time.nsec*10^(-9);
    if i > 1
        if samples{i}.action.flipper_mode ~= samples{i-1}.action.flipper_mode;
            mode_change(i) = samples{i}.action.flipper_mode+1;
        else
            mode_change(i) = 0;
        end
    else
        mode_change(i) = samples{i}.action.flipper_mode+1;

    end
    
end

annot = struct('annot', A, 'name', name, 'mode_c', mode_c, 'time_stamps', time_stamps, 'mode_change', mode_change);


end