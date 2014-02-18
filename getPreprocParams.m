function par = getPreprocParams( string_id )

    par = getDefaultPreprocParams;
    
    switch ( lower( string_id ) )
%% data - (trajectories + annotations)
        case '20140210_stairs_climbing'
            par.data_man = '/datagrid/nifti/data/2014_AT_data/20140210_stairs_climbing/';
            par.scales = [0.9 0.8 1.1];
        case '20140210_turning_on_spot'
            par.data_man = '/datagrid/nifti/data/2014_AT_data/20140210_turning_on_spot/';
            par.scales = [0.9 1.1];
            
        case '20140217_corridor'
            par.data_man = '/datagrid/nifti/data/2014_AT_data/20140217_corridor/';
            par.scales = [0.9 0.8 1.1];
            
        case '20140218_stairs_climbing'
            par.data_man = '/datagrid/nifti/data/2014_AT_data/20140218_stairs_climbing/';
            par.scales = [0.9 0.8 1.1];
            
%% episodes (trajectories only)
        case '20140128_prato_single_obst'
            par.data_man = '/datagrid/nifti/data/2014_AT_episodes/20140128_prato_single_obst/';
            par.episodic = 1;
        case '20140128_prato_roof'
            par.data_man = '/datagrid/nifti/data/2014_AT_episodes/20140128_prato_roof/';
            par.episodic = 1;
        case '20140129_prato_roof'
            par.data_man = '/datagrid/nifti/data/2014_AT_episodes/20140129_prato_roof/';
            par.episodic = 1;
        case '20140210_along_chairs'
            par.data_man = '/datagrid/nifti/data/2014_AT_episodes/20140210_along_chairs/';
            par.episodic = 1;

%% otherwise            
        otherwise
            fprintf('Unknown id of the preprocess. The default values will be used\n');
    end
end
    