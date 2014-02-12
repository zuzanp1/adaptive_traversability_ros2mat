function [par] = createComplementaryAnnot( par )


%% Load data
for k = 1 : length( par.maneuvers ),
    d0 = dir(fullfile( par.data_man, par.maneuvers(k).name, 'samples.mat') );
    par.data_mat = fullfile(par.data_man,par.maneuvers(k).name, d0(1).name);
    data(k) = load(par.data_mat);
end;

fprintf('Creating annotation');
for i = 1:length(data)
    fprintf('.');
    samples = data(i).samples;
    name = par.maneuvers(i).name;
    annot_data(i) = getComplementaryAnnot( samples, name );
end
fprintf('Done\n');

par.annot = fullfile( par.data_man, 'annotation.mat');
fprintf('Saved to: %s\n', par.annot);
save( par.annot, 'annot_data', 'par');


end