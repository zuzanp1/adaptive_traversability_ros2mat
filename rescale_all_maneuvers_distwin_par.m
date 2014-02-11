function par = rescale_all_maneuvers_distwin_par( par )
%% preprocess all batch


VISU = par.visu;
SCALES = par.scales;

%% init options
options.key.head            = 0;
options.key.left_currents   = 1;
options.key.right_currents  = 2;
options.key.eulers          = 3;
options.key.lin_vel         = 4;
options.key.ang_vel         = 5;
options.key.pose            = 6;
options.key.lower_b         = 7;
options.key.upper_b         = 8;
options.key.left_state_vel  = 9;
options.key.right_state_vel = 10;
options.key.joy_velocity    = 11; 
options.key.odom_velocity_lin= 12;
options.key.cmd_torque      = 13;
options.key.diff_brake      = 14;%CONTAINS ALSO emergency_button_on and scanning_speed
options.key.odom_velocity_ang=15;
options.key.data            = -1;
options.key.data_under      = -2;
options.key.data_rear       = -3;
options.key.narr_data       = -4;
options.key.narr_data_under = -5;
options.key.narr_data_rear  = -6;

for SCALE = SCALES
    options.z_limit             = 1.4;%will be removed in future releases, depends on PCL_MAX_Z in definitions.h (we do not want to consider all z values)

    options.bs                  = 0.1;
    options.distance_window     = par.distance_window;%[meters]
    options.neg_distance_window = par.neg_distance_window;%[meters]
    options.scale               = SCALE;
    options.offset              = 0.07;%base_link offset w.r.t to ground.
    %% 
    %prefix = '/datagrid/nifti/data/20130801_yard_pallets/';%%FIRST BATCH
    %prefix = '/datagrid/nifti/data/20130814_yard_pallets/';%SECOND BATCH
    %prefix = '/datagrid/nifti/data/20130903_yard_pallets_stairs/';%THIRD
    %BATCH
    %prefix = '/datagrid/nifti/data/20131210_E122_facing_wall/';%4th facing wall
    %prefix = '/datagrid/nifti/data/20131210_E122_facing_wall/positive/';%4rd facing the wall
    %prefix = '/datagrid/nifti/data/20131210_E122_facing_wall/negative/';options.distance_window = inf;options.neg_distance_window = inf;%4rd facing the wall
    %prefix = '/datagrid/nifti/data/20131212_E122_facing_wall_no_vel/';%5th facing wall
    
    %prefix = '~/fuerte_workspace/sandbox/nifti_adaptive_traversability/matlab/';
    %prefix = '/datagrid/nifti/data/20130906_AT_episodes/episode001/';
    prefix = par.data_man;
    if VISU
        hfig = figure; 
    end
    maneuvers = par.maneuvers;%select_file_type_with_prefix( dir(prefix), {'0', '1', '2', '3','4', '5', '6', '7', '8', '9'}, {'maneuver'});% maneuvers = maneuvers(3:end);
    for m = 1:length(maneuvers)
        mname = fullfile(prefix, maneuvers(m).name, filesep);
        mname_out = fullfile(prefix, 'scaled', sprintf('%s_scale%.3dDEM%s', maneuvers(m).name, SCALE*100));
        mkdir(mname_out);
        %load timestamps
        d = load( fullfile(mname, 'timestamps.txt' ) );%final position should be included here
        flag = d(end);
    %    thresh_sec = d(end - 2);
        open = true;
        data = select_file_type_with_prefix(dir(mname),{'txt'},{'floor'});
        collector = cell( 1,length(data) );
        sensor = nan(23, length(data));
        samples = nan(10,length(data));%each sample consist of 9 values = [robot pitch; real_linear_velocity(x,y,z); real_angular_velocity(x,y,z); flippers_mode; joy_velocity_right_left; joy_velocity_front_rear]
        %for each item do
        for i = length(data):-1:1%IMPORTATNT ITERATE FROM THE LAST ONE TO THE FIRST ONE
            d = load( fullfile(mname, data(i).name) );
            if VISU
                [desc, pcl] = preprocess_and_repair_data(d, options);
                set(0, 'currentFigure', hfig);
                col = zeros(size(pcl));
                col(:,2) = 1-(pcl(:,2) + 0.25)/0.5;
                subplot(1,2,1);plot3(pcl(:,1), pcl(:,2), pcl(:,3), 'g*');
                xlabel('x');ylabel('y');zlabel('z');
                view(0,0);
                axis equal;
                subplot(1,2,2);plot3(pcl(:,1), pcl(:,2), pcl(:,3), 'g*');xlabel('x');ylabel('y');zlabel('z');
                view(0,90);
                axis equal;
                saveas(hfig, sprintf('%s%s.jpg', mname, data(i).name(1:end-4)), 'jpg');
                clf(hfig);
            else
                desc = preprocess_and_repair_data(d, options);
            end
            desc.is_final = 0;%indicates whether the sample is the last in the sequence
            if ( open )
                term_position = desc.pose_xyz;
                term_mode = desc.y;
                desc.is_final = 1;%this is the last in the sequence
                open = false;
            end
            switch desc.cmd_torque_front %suppose front and rear are the same
                case 1.6
                    desc.complience_subclass = 1;
                case 2.2
                    desc.complience_subclass = 2;
                otherwise
                    fprintf('unknown complience class: %d', desc.cmd_torque_front);
            end
            %OLD 2013-Oct-28
            dt = sqrt( sum((term_position - desc.pose_xyz).^2) );%greater or equal to zero
            %if ( dt <= options.distance_window && term_mode == desc.y)%in case the observation is within the distance window from the LAST sample and the flipper mode is still same, than preserve the flag (i.e traversable/nontraversable)
            %    desc.flag = flag;
            %end OLD
            %NEW 2013-Oct-28
            if ( dt <= options.distance_window && term_mode == desc.y)%in case the observation is within the distance window from the LAST sample and the flipper mode is still same, than preserve the flag (i.e traversable/nontraversable)
                if ( dt > options.neg_distance_window && flag < 1 )%within the outer region
                    desc.flag = 0;%assign 0 if the flag is negative (we want to smooth the response to avoid similar data with different flags)
                else
                    desc.flag = flag;
                end
            %End NEW
            else%otherwise keep it as positive (because the robot could move there)
                desc.flag = 1;%otherwise, even the annotation is negative, keep positive (at the beginning of the maneuver there is no danger)
            end
            desc.name = data(i).name;
            %fprintf('Record: %s preprocessed\n', desc.name);
            sensor(:,i) = [desc.x', desc.xn', desc.roughness, desc.eulers, desc.left_currents, desc.right_currents, desc.left_state_vel, desc.right_state_vel]';
            samples(:,i) = [desc.eulers(1); desc.lin_vel'; desc.ang_vel'; desc.y; desc.cmd_linVel; desc.cmd_angVel];
            collector{i} = desc;
        end
        
        save (fullfile( mname_out, par.output_name), 'collector', 'data', 'options', 'sensor', 'samples');
        
        fprintf('%s preprocessed!\n', mname);
    end
end