
classdef Model < handle
    properties (SetAccess = private, GetAccess = public)
    	param
        time_step
        skin_faces
        skin_vertices
        skin_colors
    end

    properties (SetAccess = private, GetAccess = private)
        % listeners

    	% model variables
    	refsteer_yawvel_gain
    	refsteer_yawvel_numc
    	refsteer_yawvel_denc
    	refsteer_yawvel_numd
    	refsteer_yawvel_dend
    	refsteer_yawvel_tfc
    	refsteer_yawvel_tfd
    	refsteer_yawvel_inhist
    	refsteer_yawvel_outhist

    	refsteer_latacc_gain
    	refsteer_latacc_numc
    	refsteer_latacc_denc
    	refsteer_latacc_numd
    	refsteer_latacc_dend
    	refsteer_latacc_tfc
    	refsteer_latacc_tfd
    	refsteer_latacc_inhist
    	refsteer_latacc_outhist

    	refsteer_sideslip_gain
    	refsteer_sideslip_numc
    	refsteer_sideslip_denc
    	refsteer_sideslip_numd
    	refsteer_sideslip_dend
    	refsteer_sideslip_tfc
    	refsteer_sideslip_tfd
    	refsteer_sideslip_inhist
    	refsteer_sideslip_outhist

    	latacc_sideslip_gain
    	latacc_sideslip_numc
    	latacc_sideslip_denc
    	latacc_sideslip_numd
    	latacc_sideslip_dend
    	latacc_sideslip_tfc
    	latacc_sideslip_tfd
    	latacc_sideslip_inhist
    	latacc_sideslip_outhist

    	% auxilary variables
    	linvel

    	% input variables
    	refsteer
    	longacc

    	% output variables
    	linacc
    	angvel
    	sideslip
    end

    events
    end

    methods
    	function this = Model(time_step)
            this.time_step = time_step;
    	end

    	% function [] = delete(this)
    	% end

        function [] = load_vehicle(this, vehicle_file)
            % vehicle parameters
            this.param = YAML.read(vehicle_file);

            % calc additional param
            this.param.wheelbase = this.param.cg2axle_front + this.param.cg2axle_rear; % (m)
            this.param.under_over_gradient = this.param.corn_compliance_front - this.param.corn_compliance_rear; % (rad/g)
            this.param.characteristic_speed = sqrt(this.param.gravity*this.param.wheelbase/this.param.under_over_gradient);   
            this.param.axle_mass_front = this.param.mass * this.param.cg2axle_rear/ this.param.wheelbase; % (kg)
            this.param.axle_mass_rear = this.param.mass * this.param.cg2axle_front / this.param.wheelbase; % (kg)
            this.param.tire_corn_stiff_front = this.param.axle_mass_front * this.param.gravity / this.param.corn_compliance_front * (pi()/180) /2; % (N/deg)
            this.param.tire_corn_stiff_rear = this.param.axle_mass_rear * this.param.gravity / this.param.corn_compliance_rear * (pi()/180) /2; % (N/deg)

            % init transfer functions
            this.linvel = [this.param.speed 0 0];
            this.make_transfer_functions();

            % init skin
            this.make_skin();

            % init dynamic variables
            this.init();
        end

    	function [] = init(this)
			this.refsteer_yawvel_inhist = zeros(1,length(this.refsteer_yawvel_numd));
			this.refsteer_yawvel_outhist = zeros(1, length(this.refsteer_yawvel_dend));
			this.refsteer_latacc_inhist = zeros(1, length(this.refsteer_latacc_numd));
			this.refsteer_latacc_outhist = zeros(1, length(this.refsteer_latacc_dend));
			this.refsteer_sideslip_inhist = zeros(1, length(this.refsteer_sideslip_numd));
			this.refsteer_sideslip_outhist = zeros(1, length(this.refsteer_sideslip_dend));
			this.latacc_sideslip_inhist = zeros(1, length(this.latacc_sideslip_numd));
			this.latacc_sideslip_outhist = zeros(1, length(this.latacc_sideslip_dend));
			this.linvel = [this.param.speed 0 0];
			this.refsteer = 0;
			this.longacc = 0;
			this.linacc = [0, 0, 0];
			this.angvel = [0, 0, 0];
			this.sideslip = [0, 0, 0];
		end

    	function [] = set_linvel(this, linvel)
    		this.linvel = linvel;
            this.sideslip(3) = atan2(linvel(2), linvel(1));
    		this.make_transfer_functions();
    	end

    	function [] = set_input(this, longacc, refsteer)
            longacc = min(longacc, this.param.longacc_limit);
            longacc = max(longacc, -this.param.longacc_limit);
    		this.longacc = longacc;
            refsteer = min(refsteer, this.param.refsteer_limit);
            refsteer = max(refsteer, -this.param.refsteer_limit);
    		this.refsteer = refsteer;
    	end

    	function [] = calc(this)
    		% refsteer to yawvel
    		this.refsteer_yawvel_inhist(2:end) = ...
    			this.refsteer_yawvel_inhist(1:end-1);
    		this.refsteer_yawvel_inhist(1) = this.refsteer;

    		this.refsteer_yawvel_outhist(2:end) = ...
    			this.refsteer_yawvel_outhist(1:end-1);

    		this.refsteer_yawvel_outhist(1) = (...
    			this.refsteer_yawvel_numd * [this.refsteer_yawvel_inhist]' -...
    			this.refsteer_yawvel_dend(2:end) * [this.refsteer_yawvel_outhist(2:end)]'...
    			) / this.refsteer_yawvel_dend(1);

    		% refsteer to latacc
    		this.refsteer_latacc_inhist(2:end) = ...
    			this.refsteer_latacc_inhist(1:end-1);
    		this.refsteer_latacc_inhist(1) = this.refsteer;

    		this.refsteer_latacc_outhist(2:end) = ...
    			this.refsteer_latacc_outhist(1:end-1);

    		this.refsteer_latacc_outhist(1) = (...
    			this.refsteer_latacc_numd * [this.refsteer_latacc_inhist]' -...
    			this.refsteer_latacc_dend(2:end) * [this.refsteer_latacc_outhist(2:end)]'...
    			) / this.refsteer_latacc_dend(1);

    		% refsteer to sideslip
    		this.refsteer_sideslip_inhist(2:end) = ...
    			this.refsteer_sideslip_inhist(1:end-1);
    		this.refsteer_sideslip_inhist(1) = this.refsteer;

    		this.refsteer_sideslip_outhist(2:end) = ...
    			this.refsteer_sideslip_outhist(1:end-1);

    		this.refsteer_sideslip_outhist(1) = (...
    			this.refsteer_sideslip_numd * [this.refsteer_sideslip_inhist]' -...
    			this.refsteer_sideslip_dend(2:end) * [this.refsteer_sideslip_outhist(2:end)]'...
    			) / this.refsteer_sideslip_dend(1);

    		% refsteer to sideslip
    		this.latacc_sideslip_inhist(2:end) = ...
    			this.latacc_sideslip_inhist(1:end-1);
    		this.latacc_sideslip_inhist(1) = this.refsteer_latacc_outhist(1);

    		this.latacc_sideslip_outhist(2:end) = ...
    			this.latacc_sideslip_outhist(1:end-1);

    		this.latacc_sideslip_outhist(1) = (...
    			this.latacc_sideslip_numd * [this.latacc_sideslip_inhist]' -...
    			this.latacc_sideslip_dend(2:end) * [this.latacc_sideslip_outhist(2:end)]'...
    			) / this.latacc_sideslip_dend(1);


    		this.linacc = [this.longacc, this.refsteer_latacc_outhist(1), 0];
    		this.angvel = [0, 0, this.refsteer_yawvel_outhist(1)];
    		this.sideslip(1) = this.refsteer_sideslip_outhist(1);
            this.sideslip(2) = this.latacc_sideslip_outhist(1);
    	end

        function [linvel] = get_linvel(this)
            linvel = this.linvel;
        end

    	function [linacc] = get_linacc(this)
            linacc = this.linacc;
    	end

    	function [angvel] = get_angvel(this)
            angvel = this.angvel;
    	end

    	function [sideslip] = get_sideslip(this)
            sideslip = this.sideslip;
    	end

    	function [] = check_gain(this)
    		g = this.param.gravity;
    		hpr = this.param.hand_road_gearratio;
    		check_beta_g = this.latacc_sideslip_gain*g/pi()*180; % deg / G
			check_beta_d = this.refsteer_sideslip_gain*100/hpr; % deg / 100 deg hand
			check_ay_d = this.refsteer_latacc_gain/g*pi()/180*100/hpr; % G / deg hand
			check_r_d = this.refsteer_yawvel_gain*100/hpr; % deg/s / 100 deg hand
			u_ch = this.param.characteristic_speed;				
			% m_f = this.param.axle_mass_front;
			% m_r = this.param.axle_mass_rear;
			c_f = this.param.tire_corn_stiff_front;
			c_r = this.param.tire_corn_stiff_rear;
			disp(['latacc sideslip gain (deg/G) ', num2str(check_beta_g)]);
			disp(['refsteer sideslip gain (deg/100handdeg) ', num2str(check_beta_d)]);
			disp(['refsteer latacc gain (G/100handdeg) ', num2str(check_ay_d)]);
			disp(['refsteer yawvel gain (deg/s / 100handdeg) ', num2str(check_r_d)]);
    		disp(['equiv. front tire corn. stiffness (N/deg) ', num2str(c_f)]);
    		disp(['equiv. rear tire corn. stiffness (N/deg) ', num2str(c_r)]);
    		disp(['characteristic speed (m/s) ', num2str(u_ch)]);
    	end

    	function [] = plot_step(this)
    		figure
			subplot(2,2,1)
			step(this.refsteer_yawvel_tfc);
			hold on
			step(this.refsteer_yawvel_tfd);
			title('refsteer to yawvel')
			grid on

			subplot(2,2,2)
			step(this.refsteer_latacc_tfc);
			hold on
			step(this.refsteer_latacc_tfd);
			title('refsteer to latacc');
			grid on

			subplot(2,2,3)
			step(this.refsteer_sideslip_tfc);
			hold on
			step(this.refsteer_sideslip_tfd);
			title('refsteer to sideslip');
			grid on

			subplot(2,2,4)
			step(this.latacc_sideslip_tfc);
			hold on
			step(this.latacc_sideslip_tfd);
			title('latacc to sideslip')
			grid on
    	end

    	function [] = plot_bode(this)
    		bodeopts = bodeoptions;
    		bodeopts.Grid = 'On';
    		bodeopts.FreqUnits = 'Hz';
    		
    		% magnitude
    		bodeopts.MagVisible = 'On';
    		bodeopts.PhaseVisible = 'Off';
    		figure
    		subplot(2,2,1)
    		bode(this.refsteer_yawvel_tfc, bodeopts);
    		hold on
    		bode(this.refsteer_yawvel_tfd, bodeopts);
    		title('refsteer to yawvel')

    		subplot(2,2,2)
    		bode(this.refsteer_latacc_tfc, bodeopts);
    		hold on
    		bode(this.refsteer_latacc_tfd, bodeopts);
    		title('refsteer to latacc')

    		subplot(2,2,3)
    		bode(this.refsteer_sideslip_tfc, bodeopts);
    		hold on
    		bode(this.refsteer_sideslip_tfd, bodeopts);
    		title('refsteer to sideslip')

    		subplot(2,2,4)
    		bode(this.latacc_sideslip_tfc, bodeopts);
    		hold on
    		bode(this.refsteer_sideslip_tfd, bodeopts);
    		title('latacc to sideslip')

    		% phase
    		bodeopts.PhaseVisible = 'On';
    		bodeopts.MagVisible = 'Off';
    		figure
    		subplot(2,2,1)
    		bode(this.refsteer_yawvel_tfc, bodeopts);
    		hold on
    		bode(this.refsteer_yawvel_tfd, bodeopts);
    		title('refsteer to yawvel')

    		subplot(2,2,2)
    		bode(this.refsteer_latacc_tfc, bodeopts);
    		hold on
    		bode(this.refsteer_latacc_tfd, bodeopts);
    		title('refsteer to latacc')

    		subplot(2,2,3)
    		bode(this.refsteer_sideslip_tfc, bodeopts);
    		hold on
    		bode(this.refsteer_sideslip_tfd, bodeopts);
    		title('refsteer to sideslip')

    		subplot(2,2,4)
    		bode(this.latacc_sideslip_tfc, bodeopts);
    		hold on
    		bode(this.refsteer_sideslip_tfd, bodeopts);
    		title('latacc to sideslip') 		
    	end

    	function [] = print_continuous_transfer_functions(this)
    		this.refsteer_yawvel_tfc
    		disp(this.refsteer_yawvel_numc);
    		disp(this.refsteer_yawvel_denc);

    		this.refsteer_latacc_tfc
    		disp(this.refsteer_latacc_numc);
    		disp(this.refsteer_latacc_denc);

    		this.refsteer_sideslip_tfc
    		disp(this.refsteer_sideslip_numc);
    		disp(this.refsteer_sideslip_denc);

    		this.latacc_sideslip_tfc
    		disp(this.latacc_sideslip_numc);
    		disp(this.latacc_sideslip_denc);
    	end

    	function [] = print_discrete_transfer_functions(this)
    		this.refsteer_yawvel_tfd
    		disp(this.refsteer_yawvel_numd);
    		disp(this.refsteer_yawvel_dend);

    		this.refsteer_latacc_tfd
    		disp(this.refsteer_latacc_numd);
    		disp(this.refsteer_latacc_dend);

    		this.refsteer_sideslip_tfd
    		disp(this.refsteer_sideslip_numd);
    		disp(this.refsteer_sideslip_dend);

    		this.latacc_sideslip_tfd
    		disp(this.latacc_sideslip_numd);
    		disp(this.latacc_sideslip_dend);
    	end
    end

    methods (Access = private)
    	function [] = make_transfer_functions(this)
			% extract parameters
			m = this.param.mass;
			i_zz = this.param.yaw_inertia;
			a = this.param.cg2axle_front;
			b = this.param.cg2axle_rear;
			d_f = this.param.corn_compliance_front;
			d_r = this.param.corn_compliance_rear;
			hpr = this.param.hand_road_gearratio;
			g = this.param.gravity;
			u = this.linvel(1);
			wb = this.param.wheelbase;
			uog = this.param.under_over_gradient;

			% refsteer --> yawvel plant
			this.refsteer_yawvel_gain = u / (wb + u^2*uog/g); % rad/s / rad
			n2 = 0;
			n1 = g*a*b*u/d_f/wb;
			n0 = g^2*a*b/d_f/d_r/wb;
			d2 = i_zz*u/m;
			d1 = g*(a^2*b*m*d_r + a*d_f*(b^2*m+i_zz) + b*d_r*i_zz)/d_f/d_r/m/wb;
			d0 = g*a*b*(g*wb+u^2*uog)/d_f/d_r/u/wb;
			this.refsteer_yawvel_numc = [n2,n1,n0];
			this.refsteer_yawvel_denc = [d2,d1,d0];
			this.refsteer_yawvel_tfc = tf([n2,n1,n0],[d2,d1,d0]);
			this.refsteer_yawvel_tfd = c2d(this.refsteer_yawvel_tfc,...
                this.time_step, 'zoh');
			[numd, dend] = tfdata(this.refsteer_yawvel_tfd);
			this.refsteer_yawvel_numd = cell2mat(numd);
			this.refsteer_yawvel_dend = cell2mat(dend);

			% refsteer --> latacc plant
			this.refsteer_latacc_gain = u^2 / (wb + u^2*uog/g); %  m/s2 / rad
			n2 = g*b*u*i_zz/d_f/m/wb;
			n1 = g^2*a*b^2/d_f/d_r/wb;
			n0 = g^2*a*b*u/d_f/d_r/wb;
			d2 = i_zz*u/m;
			d1 = g*(a^2*b*m*d_r + a*d_f*(b^2*m+i_zz) + b*d_r*i_zz)/d_f/d_r/m/wb;
			d0 = g*a*b*(g*wb+u^2*uog)/d_f/d_r/u/wb;
			this.refsteer_latacc_numc = [n2,n1,n0];
			this.refsteer_latacc_denc = [d2,d1,d0];
			this.refsteer_latacc_tfc = tf([n2,n1,n0],[d2,d1,d0]);
			this.refsteer_latacc_tfd = c2d(this.refsteer_latacc_tfc,...
                this.time_step, 'zoh');
			[numd, dend] = tfdata(this.refsteer_latacc_tfd);
			this.refsteer_latacc_numd = cell2mat(numd);
			this.refsteer_latacc_dend = cell2mat(dend);

			% refsteer --> sideslip plant
			this.refsteer_sideslip_gain = (b - d_r*u^2/g) / (wb + u^2*uog/g); % r / r
			n2 = 0;
			n1 = g*b*i_zz/d_f/m/wb;
			n0 = g*a*b*(g*b-d_r*u^2)/d_f/d_r/u/wb;
			d2 = i_zz*u/m;
			d1 = g*(a^2*b*m*d_r + a*d_f*(b^2*m+i_zz) + b*d_r*i_zz)/d_f/d_r/m/wb;
			d0 = g*a*b*(g*wb+u^2*uog)/d_f/d_r/u/wb;
			this.refsteer_sideslip_numc = [n2,n1,n0];
			this.refsteer_sideslip_denc = [d2,d1,d0];
			this.refsteer_sideslip_tfc = tf([n2,n1,n0],[d2,d1,d0]);
			this.refsteer_sideslip_tfd = c2d(this.refsteer_sideslip_tfc,...
                this.time_step, 'zoh');
			[numd, dend] = tfdata(this.refsteer_sideslip_tfd);
			this.refsteer_sideslip_numd = cell2mat(numd);
			this.refsteer_sideslip_dend = cell2mat(dend);

			% latacc --> sideslip plant
			this.latacc_sideslip_gain = b/u^2 - d_r/g; % rad / m/s2
			n2 = 0;
			n1 = i_zz*u*d_r;
			n0 = a*m*(g*b-d_r*u^2);
			d2 = i_zz*u^2*d_r;
			d1 = g*a*b*m*u;
			d0 = g*a*m*u^2;
			this.latacc_sideslip_numc = [n2,n1,n0];
			this.latacc_sideslip_denc = [d2,d1,d0];
			this.latacc_sideslip_tfc = tf([n2,n1,n0],[d2,d1,d0]);
			this.latacc_sideslip_tfd = c2d(this.latacc_sideslip_tfc,...
                this.time_step, 'zoh');
			[numd, dend] = tfdata(this.latacc_sideslip_tfd);
			this.latacc_sideslip_numd = cell2mat(numd);
			this.latacc_sideslip_dend = cell2mat(dend);
    	end

        function [] = make_skin(this)
            w = this.param.skin_width;
            h = this.param.skin_height;
            a = this.param.skin_length * this.param.cg2axle_front / this.param.wheelbase;
            b = this.param.skin_length * this.param.cg2axle_rear / this.param.wheelbase;

            % up front
            uf_vertices = [-0 -w/2 h; a -w/2 h; a w/2 h; -0 w/2 h];
            uf_color = [0 1 0];

            % up back
            ub_vertices = [-b -w/2 h; 0 -w/2 h; 0 w/2 h; -b w/2 h];
            ub_color = [1 0 0];

            % front
            f_vertices = [a -w/2 0; a w/2 0; a w/2 h; a -w/2 h];
            f_color = [0 1 0];

            % back
            b_vertices = [-b -w/2 0; -b w/2 0; -b w/2 h; -b -w/2 h];
            b_color = [0 0 1];

            % right
            r_vertices = [-b -w/2 0; a -w/2 0; a -w/2 h; -b -w/2 h];
            r_color = [0 0 1];

            % left
            l_vertices = [-b w/2 0; a w/2 0; a w/2 h; -b w/2 h];
            l_color = [0 0 1];

            % down
            d_vertices = [-b -w/2 0; a -w/2 0; a w/2 0; -b w/2 0];
            d_color = [0 0 0];

            % combine
            this.skin_vertices = [uf_vertices;...
                ub_vertices;...
                f_vertices;...
                b_vertices;...
                r_vertices;...
                l_vertices;...
                d_vertices];
            this.skin_colors = [uf_color;...
                ub_color;...
                f_color;...
                b_color;...
                r_color;...
                l_color;...
                d_color];
            m = size(this.skin_colors,1); % num faces
            n = 4; % num vertices per face
            this.skin_faces = zeros(m,n);
            k = 1;
            for i = 1:m
                for j = 1:n
                    this.skin_faces(i,j) = k;
                    k = k+1;
                end
            end
        end
    end

    methods (Static)
	end   


end

