function [desc, varargout] = preprocess_and_repair_data(data, options)

%labels:
%   0 - [sec, nsec, state]
%   1 - [cRL, cTL, cFL] currents LEFT
%   2 - [cRR, cTR, cFR] currents RIGHT
%   3 - [roll, pitch, yaw] - euler angles
%   4 - [p.x, p.y, p.z] - robot pose
%   5 - [x,y,z] - linear velocity
%   6 - [x,y,z] - angular velocity
%   7 - [mn.x,mn.y,mn.z] - lower bound of the points in pcl taken into
%   account
%   8 - [mx,x,mx.y,mx.z] - upper bound of the points in pcl taken into
%   9 - [sRL, sFL, left_vel] - angle of rear / front left flippers + left track
%   velocity
%   10 - [sRR, sFR, right_vel] - angle of rear / front right flippers +
%   11 - [angVel, angLin, 0] - wanted angular and linear velocity
%   (control input read on joystick)
%   right track velocity
%   account
%   -1  [x,y,z] - data

if ~isfield( options, 'offset' )
    options.offset = 0.07;
end
if ~isfield( options, 'scale' )
    options.scale = 1;
end

%parse data
u = data(:,1);
uu = unique( u )';
lsqxy = [];
nlsqxy = [];
lsqz = [];
nlsqz = [];
desc = [];
for i = uu 
    select = data(u==i,2:end);%choose only appropriate data
    switch(i)
        case options.key.head
            y = select(end);
            t.sec = select(1);
            t.nsec = select(2);
        case options.key.left_currents
            left_currents = select;
        case options.key.right_currents
            right_currents = select;
        case options.key.eulers
            eulers = select;
        case options.key.pose
            pose_xyz = select;
        case options.key.lin_vel
            lin_vel = select;
        case options.key.ang_vel
            ang_vel = select;
        case options.key.lower_b
            min_x = select(1);
            min_y = select(2);
        case options.key.upper_b
            max_x = select(1);
            max_y = select(2);
        %case options.key.joy_velocity
        %    joy_velocity = select(1:2);
        case options.key.data
            xy = select(:,[1,2]);
            z = select(:,3);
            lsqxy = [lsqxy; select(:,[1,2])];
            lsqz = [lsqz; select(:,3)];
        case options.key.data_under
            lsqxy = [lsqxy; select(:,[1,2])];
            lsqz = [lsqz; select(:,3)];
        case options.key.data_rear
            lsqxy = [lsqxy; select(:,[1,2])];
            lsqz = [lsqz; select(:,3)];
        case options.key.left_state_vel
            left_state_vel = select;
        case options.key.right_state_vel
            right_state_vel = select;
        case options.key.joy_velocity
            cmd_linVel = select(1);
            cmd_angVel = select(2);
        case options.key.odom_velocity_lin
            odom_linVel = select;
        case options.key.odom_velocity_ang
            odom_angVel = select;
        case options.key.cmd_torque
            cmd_torque_front = select(1);
            cmd_torque_rear = select(2);
        case options.key.diff_brake
            diff_brake_on = select(1);
            emerg_btn_on = select(2);
        case options.key.narr_data
            nlsqxy = [nlsqxy; select(:,[1,2])];
            nlsqz = [nlsqz; select(:,3)];
        case options.key.narr_data_under
            nlsqxy = [nlsqxy; select(:,[1,2])];
            nlsqz = [nlsqz; select(:,3)];
        case options.key.narr_data_rear
            nlsqxy = [nlsqxy; select(:,[1,2])];
            nlsqz = [nlsqz; select(:,3)];
                
    end
end %end of parsing
if ( isfield(options, 'z_limit') )
    %m = z >= - options.z_limit & z < options.z_limit;
    %xy = xy(m,:);
    %z = z(m);
    m = lsqz >= - options.z_limit & lsqz < options.z_limit;
    lsqxy = lsqxy(m,:);
    lsqz = lsqz(m);
    
end

%%NEW 203-Oct-28lsqz = rescale_features( lsqz, options.scale, options.offset);%rescale pts
%%NEW 203-Oct-28nlsqz = rescale_features( nlsqz, options.scale, options.offset);%rescale normalized pts

if ~isempty(lsqz) && ~isempty(nlsqz)

    %%NEW 2013-Oct-28
    roll = eulers(1)*pi/180;
    pitch = eulers(2)*pi/180;
    %yaw = eulers(3)*pi/180;

    T = getTransform( roll, pitch, 0, 0, 0, 0);%pose_xyz(1), pose_xyz(2), pose_xyz(3) );
    nT = getTransform( roll, 0, 0, 0, 0, 0);%pose_xyz(1), pose_xyz(2), pose_xyz(3) );

    tlsq = transformPoints( inv(T), [lsqxy,lsqz]' );
    ntlsq = transformPoints( inv(nT), [nlsqxy, nlsqz]' );

    tlsq(end,:) = rescale_features( tlsq(end,:), options.scale, options.offset);
    ntlsq(end,:) = rescale_features( ntlsq(end,:), options.scale, options.offset);

    lsqxyz = transformPoints(T, tlsq );
    nlsqxyz = transformPoints(nT, ntlsq );

    lsqxy = lsqxyz(1:2,:)'; lsqz = lsqxyz(3,:)';
    nlsqxy = nlsqxyz(1:2,:)'; nlsqz = nlsqxyz(3,:)';

    eulers(1) = rescale_features( eulers(1), options.scale, 0);%rescale roll

    %%end NEW

    eulers(2) = rescale_features( eulers(2), options.scale, 0);%rescale pitch


    %desc = cell_gen(xy,z,min_x,max_x,min_y,max_y,options.bs);
    [lsq.x, lsq.xn, lsq.roughness] = lsq_plane([lsqxy,lsqz]);
    desc = cell_gen_and_repair(lsqxy, lsqz, min_x, max_x, min_y, max_y, options.bs, lsq);
    if ~isempty(nlsqz) || all(lsq.x == 0)
        [nlsq.x, nlsq.xn, nlsq.roughness] = lsq_plane([nlsqxy,nlsqz]);
        ndesc = cell_gen_and_repair(nlsqxy, nlsqz, min_x, max_x, min_y, max_y, options.bs, nlsq);
        desc.nDEM = ndesc.DEM;
        desc.nh = ndesc.h;
        desc.nterrain = ndesc.terrain;
        desc.nmean_terrain = ndesc.mean_terrain;
        desc.nvar_terrain = ndesc.var_terrain;
        desc.nmissing = ndesc.missing;%this is scalar
        desc.nincomplete_terrain = ndesc.incomplete_terrain;
        desc.noccupacy = ndesc.occupacy;
        desc.nx = nlsq.x;
        desc.nxn = nlsq.xn;
        desc.nroughness = nlsq.roughness;
        %perturbation
        if isfield( options, 'perturbType' ) && options.perturbType
            desc.nDEM = perturbDEM( ndesc.DEM, options.perturbType);
            desc.nh = perturbDEM( ndesc.h, options.perturbType);
            desc.nterrain = perturbDEM( ndesc.terrain, options.perturbType);
            desc.nmean_terrain = perturbDEM( ndesc.mean_terrain, options.perturbType);
            desc.nvar_terrain = perturbDEM( ndesc.var_terrain, options.perturbType);
            desc.nincomplete_terrain = perturbDEM( ndesc.incomplete_terrain, options.perturbType);
            desc.noccupacy = perturbDEM( ndesc.occupacy, options.perturbType );
            [desc.nx, desc.nxn, desc.nroughness] = lsq_from_dem( desc.nmean_terrain, desc.noccupacy, min_y, min_x, options.bs);%min_x -> min_y image->base_link
        end
    end
    desc.x = lsq.x;
    desc.xn = lsq.xn;
    desc.roughness = lsq.roughness;
    %perturbation
    if isfield( options, 'perturbType' ) && options.perturbType
        desc.DEM = perturbDEM( desc.DEM, options.perturbType);
        desc.h = perturbDEM( desc.h, options.perturbType);
        desc.terrain = perturbDEM( desc.terrain, options.perturbType);
        desc.mean_terrain = perturbDEM( desc.mean_terrain, options.perturbType);
        desc.var_terrain = perturbDEM( desc.var_terrain, options.perturbType);
        desc.incomplete_terrain = perturbDEM( desc.incomplete_terrain, options.perturbType);
        desc.occupacy = perturbDEM( ndesc.occupacy, options.perturbType );
        [desc.x, desc.xn, desc.roughness] = lsq_from_dem( desc.mean_terrain, desc.occupacy, min_y, min_x, options.bs);%min_x -> min_y image->base_link
    end

    desc.y = y;
    desc.eulers = eulers;
    desc.left_currents = left_currents;
    desc.right_currents = right_currents;
    desc.left_state_vel = left_state_vel;
    desc.right_state_vel = right_state_vel;
    desc.t = t;
    desc.pose_xyz = pose_xyz;
    desc.lin_vel = lin_vel;
    desc.ang_vel = ang_vel;
    %desc.joy_velocity = joy_velocity;

    desc.cmd_linVel = cmd_linVel;
    desc.cmd_angVel = cmd_angVel;

    desc.odom_linVelX = odom_linVel(1);
    desc.odom_linVelY = odom_linVel(2);
    desc.odom_linVelZ = odom_linVel(3);
    desc.odom_angVelX = odom_angVel(1);
    desc.odom_angVelY = odom_angVel(2);
    desc.odom_angVelZ = odom_angVel(3);
    %%new added for perturbDEM on Nov 15, 2013
    desc.min_x = min_x;
    desc.min_y = min_y;
    desc.max_x = max_x;
    desc.max_y = max_y;
    desc.bin_size = options.bs;
    %%end added
    desc.cmd_torque_front = cmd_torque_front;
    desc.cmd_torque_rear = cmd_torque_rear;
    if exist('diff_brake_on', 'var')
        desc.diff_brake_on = diff_brake_on;
        desc.emerg_btn_on = emerg_btn_on;
    end
    
end%if ~isempty(lsqz) && ~isempty(nlsqz)



if nargout > 1
    varargout(1) = {[lsqxy,lsqz]};
end
end


function [pts_out] = rescale_features(pts_in, scale, offset)

    pts_out = (pts_in + offset)*scale - offset;
    
end

function R = setRPY( roll, pitch, yaw )

%void setRPY(const tfScalar& roll, const tfScalar& pitch, const tfScalar& yaw)
%	{
		halfYaw = yaw * 0.5;  
		halfPitch = pitch * 0.5;  
		halfRoll = roll * 0.5;  
		cosYaw = cos(halfYaw);
		sinYaw = sin(halfYaw);
		cosPitch = cos(halfPitch);
		sinPitch = sin(halfPitch);
		cosRoll = cos(halfRoll);
		sinRoll = sin(halfRoll);
		q = [sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;...% //x
                         cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;...% //y
                         cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;...% //z
                         cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw];...% //formerly yzx
%   }
%void setRotation(const Quaternion& q) 
%	{
		d = q'*q;%length2
		%tfFullAssert(d != tfScalar(0.0));
		s = 2.0 / d;
		xs = q(1) * s;   ys = q(2) * s;   zs = q(3) * s;
		wx = q(4) * xs;  wy = q(4) * ys;  wz = q(4) * zs;
		xx = q(1) * xs;  xy = q(1) * ys;  xz = q(1) * zs;
		yy = q(2) * ys;  yz = q(2) * zs;  zz = q(3) * zs;
		R = [1.0 - (yy + zz), xy - wz, xz + wy;
			xy + wz, 1.0 - (xx + zz), yz - wx;
			xz - wy, yz + wx, 1.0 - (xx + yy)];
%	}
end

function t = setOrigin(x, y, z)
    t = [x;y;z];
end

function T = getTransform( roll, pitch, yaw, x, y, z)
    T = [setRPY(roll, pitch, yaw), setOrigin(x,y,z);
         0    0    0   1];
end

function pts_o = transformPoints( T, pts )
    pts = [pts;ones(1, size(pts,2))];
    pts_o = T*pts;
    
    pts_o = bsxfun(@rdivide, pts_o(1:end-1,:), pts_o(end,:));
    
    
end