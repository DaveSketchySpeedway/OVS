classdef Robot < handle
    properties (SetAccess = private, GetAccess = public)    
        time_step
        trajectory
        look_ahead
        search_range
        steer_gain
    end

    properties (SetAccess = private, GetAccess = private)
        measured_position
        reference_position
        ahead_position
        body_poserr_vector
        body_ahead_vector
        body_ahead_rotation
        body_linvel
        current_trajectory_index
        ahead_trajectory_index
    end

    events
    end

    methods
    	function this = Robot(time_step)
            this.time_step = time_step;
            this.look_ahead = 10;
            this.search_range = 20;
            this.steer_gain = 1.6;
    	end

    	% function [] = delete(this)
    	% end

        function [] = init(this)
            this.current_trajectory_index = 1;
        end

        function [] = load_controller(this, controller_path)
        end

        function [] = set_trajectory(this, trajectory)
            this.trajectory = trajectory;
        end

        function [] = find_reference(this, measured_position)
            this.measured_position = measured_position;
            distance = 9999;
            for i =...
                max(1, this.current_trajectory_index-this.search_range):...
                length(this.trajectory.splinepoints)
                % min(length(this.trajectory.splinepoints), this.current_trajectory_index+this.search_range)
                d = norm(this.measured_position -...
                    this.trajectory.splinepoints(i,1:3));
                if d <= distance
                    distance = d;
                else
                    this.current_trajectory_index = i;
                    break;
                end
            end
            this.reference_position = ...
                this.trajectory.splinepoints(this.current_trajectory_index,1:3);
            this.ahead_trajectory_index = this.current_trajectory_index +...
                this.look_ahead;
            this.ahead_trajectory_index = min(this.ahead_trajectory_index,...
                length(this.trajectory.splinepoints));
            this.ahead_position = this.trajectory.splinepoints(...
                this.ahead_trajectory_index,1:3);
        end

        function [vector] = get_poserr_vector(this)
            vector = this.reference_position - this.measured_position;
        end

        function [vector] = get_ahead_vector(this)
            vector = this.ahead_position - this.measured_position;
        end

        function [] = set_body_poserr_vector(this, body_poserr_vector)
            this.body_poserr_vector = body_poserr_vector;
        end

        function [] = set_body_ahead_vector(this, body_ahead_vector)
            this.body_ahead_vector = body_ahead_vector;
            this.body_ahead_rotation(1) = 0;
            this.body_ahead_rotation(2) = atan2(-body_ahead_vector(3),...
                body_ahead_vector(1));
            this.body_ahead_rotation(3) = atan2(body_ahead_vector(2),...
                body_ahead_vector(1));
        end

        function [throttle, joy_x] = get_command(this)
            throttle = 0;
            joy_x = this.steer_gain * this.body_ahead_rotation(3);
        end

    end

    methods (Access = private)
    end

    methods (Static)
	end   


end

