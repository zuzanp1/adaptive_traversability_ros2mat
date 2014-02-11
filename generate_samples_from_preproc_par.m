function par = generate_samples_from_preproc_par( par )
%% describe all batch

preproc = par.output_name;

UNCERTAINTY = par.distance_window > par.neg_distance_window;%soft assigment of the final reward on the reward boundaris [1 1 1-1 -1 -1] -> [1 0 0 0 0-1]

%prefix = '/datagrid/nifti/data/20130801_yard_pallets/';%BATCH 01
%prefix = '/datagrid/nifti/data/20130801_yard_pallets/scaled/';
%prefix = '/datagrid/nifti/data/20130801_yard_pallets/perturbed/';
%prefix = '/datagrid/nifti/data/20130814_yard_pallets/';%BATCH 02
%prefix = '/datagrid/nifti/data/20130814_yard_pallets/scaled/';
%prefix = '/datagrid/nifti/data/20130903_yard_pallets_stairs/';%BATCH 3
%prefix = '/datagrid/nifti/data/20130903_yard_pallets_stairs/scaled/'
%prefix = '/datagrid/nifti/data/20131210_E122_facing_wall/';%BATCH 4
%prefix = '/datagrid/nifti/data/20131210_E122_facing_wall/positive/';%4rd facing the wall
%prefix = '/datagrid/nifti/data/20131210_E122_facing_wall/negative/';preproc='preproc_data_bs1_repaired_distancewindowInfinInf.mat';options.distance_window = inf;options.neg_distance_window = inf;%4rd facing the wall
%prefix = '/datagrid/nifti/data/20131210_E122_facing_wall/positive/scaled/';%4rd facing the wall
%prefix = '/datagrid/nifti/data/20131210_E122_facing_wall/negative/scaled/';preproc='preproc_data_bs1_repaired_distancewindowInf.mat';options.distance_window = inf;options.neg_distance_window = inf;%4rd facing the wall
%prefix = '/datagrid/nifti/data/20131210_E122_facing_wall/scaled/';%BATCH 4 scaled
%prefix = '/datagrid/nifti/data/20131212_E122_facing_wall_no_vel/';%BATCH 5
%prefix = '/datagrid/nifti/data/20131212_E122_facing_wall_no_vel/scaled/';%BATCH 5 scaled

%prefix = '/datagrid/nifti/data/20130814_yard_pallets/perturbed/';
%prefix = '/datagrid/nifti/data/20130814_yard_pallets_scaled/';%THESE ARE
%OLD-ONES before 2013-Oct-28
%prefix = '/datagrid/nifti/data/20130801_yard_pallets_scaled/';%THESE ARE
%OLD-ONES before 2013-Oct-28
%prefix = '/datagrid/nifti/data/20130826_yard_pallets_pointmap_scaled/'; 

%prefix = '/datagrid/nifti/data/20130906_AT_episodes/episode001/scaled/';
%%prefix = '/datagrid/nifti/data/20130904_AT_krc_forest/icra14_manual/';
%prefix = '/datagrid/personal/zuzanpet/20131115_data_perturb_test/perturbed/';

%%%%ICRA 2014experiments -extended
%preproc = 'preproc_data_bs1_repaired_distancewindow030.mat';
%UNCERTAINTY = 0;
%prefix = '/datagrid/nifti/data/20140117_AT_ICRA2014_experiments/manual/';
%prefix = '/datagrid/nifti/data/20140117_AT_ICRA2014_experiments/semiauto/';
%prefix = '/datagrid/nifti/data/20140117_AT_ICRA2014_experiments/auto/';

%%PRATO 2014 experiments alongside final review
%prefix = '/datagrid/nifti/data/201401_review/20140128_AT_prato/manual/';
%prefix = '/datagrid/nifti/data/201401_review/20140128_AT_prato/semiauto/';
%prefix = '/datagrid/nifti/data/201401_review/20140129_AT_prato/manual/';
%prefix = '/datagrid/nifti/data/201401_review/20140129_AT_prato/semiauto/';
%prefix = '/datagrid/nifti/data/201401_review/20140128_AT_prato/experiment_032prato/';
%%%%
prefix = par.data_man;

SPLIT_TIP_OVER_MODE = 1;
TIP_OVER = 4;
RAD_TO_DEG = 180/pi;

maneuvers = par.maneuvers;


%descriptor handle
%descriptor = @desc_haar;
for m = 1:length(maneuvers)
    mname = fullfile(prefix, maneuvers(m).name, filesep);
    if UNCERTAINTY
        out_mname = fullfile( prefix, 'uncert', maneuvers(m).name, filesep);
        mkdir( out_mname )
    else
        out_mname = mname;
    end
    
    load(fullfile( mname, preproc) );
    L = length(collector);

    samples = cell(1,L);
    pt = 1;
    for i = 1:L
        desc = collector{i};
        if isempty(desc)%if the sample is corrupted - desc is empty, we want to skip that
            fprintf('Sample description is missing..skipping\n');
            continue;
        end
        if ~isfield('diff_brake_on', desc)
            differential_brake_on = 1;
        end
        mode = desc.y;
        if (SPLIT_TIP_OVER_MODE && mode == TIP_OVER && desc.complience_subclass == 2)
            mode = mode+1;
        end
            
        sample = struct('state', struct('DEM', desc.DEM, 'missing_values', desc.incomplete_terrain, 'DEM_variance', desc.var_terrain, 'occupacy', desc.occupacy),...
                        'nstate', struct('DEM', desc.nDEM, 'missing_values', desc.nincomplete_terrain, 'DEM_variance', desc.nvar_terrain, 'occupacy', desc.noccupacy),...    
                        'action', struct('flipper_mode', mode, 'compliance_front', desc.cmd_torque_front, 'compliance_rear', desc.cmd_torque_rear,...
                        'differential_brake_on', differential_brake_on, 'linVel', desc.cmd_linVel, 'angVel', desc.cmd_angVel),...
                        'sensors', struct('LSQ_DEM_abcd', desc.xn, 'LSQ_DEM_kqr', desc.x, 'LSQ_error', desc.roughness,...
                        'nLSQ_DEM_abcd', desc.nxn, 'nLSQ_DEM_kqr', desc.nx, 'nLSQ_error', desc.nroughness,'roll', desc.eulers(1), 'pitch', desc.eulers(2),...
                        'linVelX_imu', desc.odom_linVelX, 'linVelY_imu', desc.odom_linVelY, 'linVelZ_imu', desc.odom_linVelZ,...
                        'angVelX_imu', desc.odom_angVelX, 'angVelY_imu', desc.odom_angVelY, 'angVelZ_imu', desc.odom_angVelZ,...
                        'current_LeftRear', desc.left_currents(1), 'current_LeftMain', desc.left_currents(2),'current_LeftFront', desc.left_currents(3),...
                        'current_RightRear', desc.right_currents(1), 'current_RightMain', desc.right_currents(2),'current_RightFront', desc.right_currents(3),...
                        'angle_LeftRear', desc.left_state_vel(1)*RAD_TO_DEG, 'angle_LeftFront', desc.left_state_vel(2)*RAD_TO_DEG,...
                        'angle_RightRear', desc.right_state_vel(1)*RAD_TO_DEG, 'angle_RightFront', desc.right_state_vel(2)*RAD_TO_DEG ),...
                        'final_reward', desc.flag, 'source_name', sprintf('%s/%s', maneuvers(m).name, desc.name), 'time', struct('sec', desc.t.sec, 'nsec', desc.t.nsec),...
                        'is_final', desc.is_final);
                    
            
        fprintf('RECORD %s transformed to sample\n', desc.name);
        samples{pt} = sample;
        pt = pt+1;
    end
    samples = samples(1:pt-1);
    
    fprintf('%s described succesfully\n', out_mname);
    save( fullfile(out_mname, 'samples.mat'), 'samples' );
    
end
