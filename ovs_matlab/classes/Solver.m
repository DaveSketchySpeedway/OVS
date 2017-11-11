classdef Solver < handle
    properties (SetAccess = private, GetAccess = public)
    time_step
    model
    end

    properties (SetAccess = private, GetAccess = private)
    % listeners

    % solver variables
    position
    linvel
    linacc
    orientation
    angvel
    end

    events
    end

    methods
    	function this = Solver(time_step)
            this.time_step = time_step;
            this.position = [0,0,0];
            this.linvel = [100/3.6,0,0];
            this.linacc = [0,0,0];
            this.orientation = [1,0,0,0];
            this.angvel = [0,0,0];
    	end

    	% function [] = delete(this)
    	% end

        function [] = set_model(this, model)
            this.model = model;
        end

        function [] = step(this)
            % update model
            body_linvel = this.transform2body(this.linvel);
            this.model.set_linvel(body_linvel);
            this.model.calc();

            % update solver
            this.set_body_linacc(this.model.get_linacc());
            this.set_body_angvel(this.model.get_angvel());
            this.update();
        end

        function [vertices] = render(this)
            vertices = this.translate_vector(...
                this.rotate_vector(this.model.skin_vertices));
        end

        function [] = init_position(this, position)
            this.position = position;
        end

        function [] = init_orientation(this, orientation)
            this.orientation = orientation;
        end

        function [] = init_linvel(this, linvel)
            this.linvel = linvel;
        end

        function [] = set_body_linacc(this, body_linacc)
            this.linacc = this.transform2local(body_linacc);
        end

        function [] = set_body_angvel(this, body_angvel)
            this.angvel = this.transform2local(body_angvel);
        end

        function [] = update(this)
            % update linear
            this.linvel = this.linvel + this.linacc .* this.time_step;
            this.position = this.position + this.linvel .* this.time_step;

            % update angular
            n = norm(this.angvel);
            if n == 0
                b = 0;
                u = this.angvel;
            else
                b = n*this.time_step;
                u = this.angvel ./ n;
            end
            rotation = [cos(b/2), u.*sin(b/2)];
            this.orientation = quaternion_mul(rotation, this.orientation);
        end

        % function [] = finalize(this)
        % end

        function [vectors] = rotate_vector(this, vectors)
            vecrotmat = quaternion2rotmat(this.orientation);
            s = size(vectors);
            for i = 1:s(1)            
                vectors(i,:) = [vecrotmat * vectors(i,:)']';
            end
        end

        function [vectors] = translate_vector(this, vectors)
            s = size(vectors);
            for i = 1:s(1)
                vectors(i,:) = vectors(i,:) + this.position;
            end
        end

        function [vectors] = transform2local(this, vectors)
            vecrotmat = quaternion2rotmat(this.orientation);
            s = size(vectors);
            for i = 1:s(1)
                vectors(i,:) = [vecrotmat * vectors(i,:)']';
            end
        end

        function [vectors] = transform2body(this, vectors)
            vecrotmat = quaternion2rotmat(this.orientation);
            s = size(vectors);
            for i = 1:s(1)
                vectors(i,:) = [vecrotmat' * vectors(i,:)']';
            end
        end

        function [position] = get_position(this)
            position = this.position;
        end

        function [linvel] = get_linvel(this)
            linvel = this.linvel;
        end

        function [linacc] = get_linacc(this)
            linacc = this.linacc;
        end

        function [orientation] = get_orientation(this)
            orientation = this.orientation;
        end

        function [angvel] = get_angvel(this)
            angvel = this.angvel;
        end
    end

    methods (Access = private)
    end

    methods (Static)
	end   


end

