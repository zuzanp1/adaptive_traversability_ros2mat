function par = getPreprocParams( string_id )

    par = getDefaultPreprocParams;
    
    switch ( lower( string_id ) )
%% data - (trajectories + annotations)
        case '20140210_stairs_climbing'
            par.data_man = '/datagrid/nifti/data/2014_AT_data/20140210_stairs_climbing/';
        case '20140210_turning_on_spot'
            par.data_man = '/datagrid/nifti/data/2014_AT_data/20140210_turning_on_spot/';

            
%% episodes (trajectories only)
        case '20140128_prato_single_obst'
            par.data_man = '/datagrid/nifti/data/2014_AT_episodes/20140128_prato_single_obst/';
            par.episodes = 1;
        case '20140128_prato_roof'
            par.data_man = '/datagrid/nifti/data/2014_AT_episodes/20140128_prato_roof/';
            par.episodes = 1;
        case '20140129_prato_roof'
            par.data_man = '/datagrid/nifti/data/2014_AT_episodes/20140129_prato_roof/';
            par.episodes = 1;

%% otherwise            
        otherwise
            fprintf('Unknown id of the preprocess. The default values will be used\n');
    end
end
    