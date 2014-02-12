%go all
for K = 1:length(KK)
    %get parameters
    par = getPreprocParams( KK{K} );
    %do preprocessing
    par = preprocess_all_maneuvers_distwin_par( par );
    %generate matlab samples for RL task
    par = generate_samples_from_preproc_par( par );
    %perturb data (scale is stored in par.scales) -  this only preprocess
    %the files.
    par = rescale_all_maneuvers_distwin_par( par );
    %generate matlab samples for RL task from perturbe preproc data
    par.scaling = generate_samples_from_preproc_par( par.scaling );
    %save the par structure in par.data_man location
    savepar;
    
end