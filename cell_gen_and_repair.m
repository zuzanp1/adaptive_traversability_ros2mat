function [desc] = cell_gen_and_repair(xy, z, xmin, xmax, ymin, ymax, bs, lsq)

%N = size(xy,1);
X_MAX = round( (xmax - xmin)/bs );
Y_MAX = round( (ymax - ymin)/bs );
if isempty(xy)
	h = zeros( X_MAX, Y_MAX );%full( sparse( [1;X_MAX], [1;Y_MAX], [0;0] ) ); 
    desc = struct('DEM', h, 'h', h, 'occupacy', h, 'terrain', h, 'mean_terrain', h, 'var_terrain', h, 'missing', sum(h(:) == 0), 'incomplete_terrain', h==1);
    return
end





m = (xy(:,1) < (xmax-eps) & xy(:,2) < (ymax-eps) );%tricky part, for sure that the values are smaller than maximal one.
m = m & (xy(:,1) > xmin & xy(:,2) > ymin);%NEW 2013-Oct-28 - since the novel rescaling procedure has been introduced
z = z(m);
X = xy( m ,1) - xmin;%shift to make it non negative
Y = xy( m ,2) - ymin;%shift to make it non negative
KX = floor(X / bs) + 1;%get keys (indexes of bins)
KY = floor(Y / bs) + 1;



if( any(KY > 5) || any(KX > 20) )
   fprintf('.'); 
end

%compute pure histogram (+1 per occurence)
h = full( sparse( [KX;1;X_MAX], [KY;1;Y_MAX], [ones(size(KX));0;0] ) ); 
occupacy = h;
%repair empty cells (substitute by lsq estimate)
%h(1,1) = h(1,2);h(1,end) = h(1,end-1);//hack for terrain in front of the
%robot only to avoid the artefacts on the main
%tracks
incomplete_terrain = h == 0;
idx = find( incomplete_terrain );
[ri, ci ] = ind2sub( size(h), idx );
rm = ri * bs + xmin - bs/2;
cm = ci * bs + ymin - bs/2;
zm = lsq.x'*[rm, cm, ones(size(rm))]';

h(idx) = 1;

%compute terrain histogram (+z per occurence)
terrain = full( sparse( [KX;1;X_MAX], [KY;1;Y_MAX], [z;0;0] ) );
terrain(idx) = zm;
%terrain(1,1) = terrain(1,2); terrain(1,end) = terrain(1,end-1);//trick for
%terrain in front of the robot only to avoid the artefacts on the main
%tracks
%compute terrain squared histogram (+z^2 per occurence) for variance
%computation
terrain2 = full( sparse( [KX;1;X_MAX], [KY;1;Y_MAX], [z.*z;0;0] ) );
terrain2(idx) = zm.*zm;
%terrain2(1,1) = terrain2(1,2); terrain2(1,end) = terrain2(1,end-1);//trick for
%terrain in front of the robot only to avoid the artefacts on the main
%tracks
%compute mean value of terrain height in each cell (1/N*sum(z)) using
%the above precomputed results
mean_terrain = terrain./h;%./max(h,eps);
%compute the variance of terrain height in each cell (1/N*(sum(z.^2) -
%N*mean.^2) using the above precomputed results.
var_terrain = abs( (terrain2./h) - mean_terrain.^2 );%should be  positive at all but there might be some small negative residuals caused by rounding precision
%make the density from occurences -> this is not a relevant measure (the
%density decreases with the distance (as far as laser scans the bigger gabs
%between the consecutive rays are)
h = h./sum(h(:));

%prepare struct containing all features computed above (except terrain2) 
desc = struct('DEM', mean_terrain + 2*sqrt(var_terrain), 'h', h, 'occupacy', occupacy, 'terrain', terrain, 'mean_terrain', mean_terrain, 'var_terrain', var_terrain, 'missing', sum(h(:) == 0), 'incomplete_terrain', incomplete_terrain);


end