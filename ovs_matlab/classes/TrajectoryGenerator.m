classdef TrajectoryGenerator < handle
    properties (SetAccess = private, GetAccess = public)    
        resolution   
        tension
        checkpoints
        splinepoints
    end

    properties (SetAccess = private, GetAccess = private)
    end

    events
    end


    methods
    	function this = TrajectoryGenerator()
            this.resolution = 100;
            this.tension = 0;
            this.checkpoints.x = [];
            this.checkpoints.y = [];
            this.checkpoints.z = [];
            this.splinepoints = [];
    	end

    	% function [] = delete(this)
    	% end

        function [] = load_path(this, path_file);
            fin = YAML.read(path_file);
            this.checkpoints = fin.checkpoints;
        end

        function [] = save_path(this, path_file);
            fout.checkpoints = this.checkpoints;
            YAML.write(path_file, fout);
        end

        function [] = insert_checkpoint(this, index, x, y, z)
        end

        function [] = delete_checkpoint(this, index)
        end

        function [] = move_checkpoint(this, index, x, y, z)
        end

        function [] = scale_all_checkpoints(this, scale_x, scale_y, scale_z)
            for i = 1:length(this.checkpoints)
                this.checkpoints(i).x = this.checkpoints(i).x * scale_x;
                this.checkpoints(i).y = this.checkpoints(i).y * scale_y;
                this.checkpoints(i).z = this.checkpoints(i).z * scale_z;
            end
        end

        function [x] = get_checkpoints_x(this)
            for i = 2:length(this.checkpoints)-1
                x(i) = this.checkpoints(i).x;
            end
        end

        function [y] = get_checkpoints_y(this)
            for i = 2:length(this.checkpoints)-1
                y(i) = this.checkpoints(i).y;
            end
        end

        function [z] = get_checkpoints_z(this)
            for i = 2:length(this.checkpoints)-1
                z(i) = this.checkpoints(i).z;
            end
        end

        function [x] = get_fringepoints_x(this)
            x = [this.checkpoints(1).x this.checkpoints(end).x];
        end

        function [y] = get_fringepoints_y(this)
            y = [this.checkpoints(1).y this.checkpoints(end).y];
        end

        function [z] = get_fringepoints_z(this)
            z = [this.checkpoints(1).z this.checkpoints(end).z];
        end

        function [] = generate(this);
            k = 1;
            for i = 2:length(this.checkpoints)-2
                x0 = this.checkpoints(i-1).x;
                y0 = this.checkpoints(i-1).y;
                z0 = this.checkpoints(i-1).z;
                x1 = this.checkpoints(i).x;
                y1 = this.checkpoints(i).y;
                z1 = this.checkpoints(i).z;
                x2 = this.checkpoints(i+1).x;
                y2 = this.checkpoints(i+1).y;
                z2 = this.checkpoints(i+1).z;
                x3 = this.checkpoints(i+2).x;
                y3 = this.checkpoints(i+2).y;
                z3 = this.checkpoints(i+2).z;
                t = linspace(0,1,this.resolution);
                mx1 = (1-this.tension)*(x2-x0)/2;
                my1 = (1-this.tension)*(y2-y0)/2;
                mz1 = (1-this.tension)*(z2-z0)/2;
                mx2 = (1-this.tension)*(x3-x1)/2;
                my2 = (1-this.tension)*(y3-y1)/2;
                mz2 = (1-this.tension)*(z3-z1)/2;
                for j = 1:length(t)-1
                    this.splinepoints(k, 1) = x1*(2*t(j)^3-3*t(j)^2+1) +...
                        x2*(-2*t(j)^3+3*t(j)^2) + ...
                        mx1*(t(j)^3-2*t(j)^2+t(j)) + ...
                        mx2*(t(j)^3-t(j)^2);
                    this.splinepoints(k, 2) = y1*(2*t(j)^3-3*t(j)^2+1) + ...
                        y2*(-2*t(j)^3+3*t(j)^2) + ...
                        my1*(t(j)^3-2*t(j)^2+t(j)) + ...
                        my2*(t(j)^3-t(j)^2);
                    this.splinepoints(k, 3) = z1*(2*t(j)^3-3*t(j)^2+1) + ...
                        z2*(-2*t(j)^3+3*t(j)^2) + ...
                        mz1*(t(j)^3-2*t(j)^2+t(j)) + ...
                        mz2*(t(j)^3-t(j)^2);
                    this.splinepoints(k, 4) = 0;
                    k = k+1;
                end
            end
        end

        function [x] = get_splinepoints_x(this)
            x = this.splinepoints(:,1);
        end

        function [y] = get_splinepoints_y(this)
            y = this.splinepoints(:,2);
        end

        function [z] = get_splinepoints_z(this)
            z = this.splinepoints(:,3);
        end

        function [v] = get_splinepoint_speed(this)
            v = this.splinepoints(:,4);
        end

    end

    methods (Access = private)
    end

    methods (Static)
	end   


end

