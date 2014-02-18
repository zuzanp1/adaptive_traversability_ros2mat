function par = getDefaultPreprocParams
% constants
    par.z_limit = 1.4;%meters
    par.bin_size = 0.1;%meters
    par.distance_window = 0.3;%meters
% variables
    par.scales = [];%no scales
    par.neg_distance_window = 0.3;% default
    par.visu = 0;
    par.episodic = 0;
    par.data_man = '/datagrid/nifti/data/20130314_yard_obstacle_maneuvers_no_bags/';

    
end