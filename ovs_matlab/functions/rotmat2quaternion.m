function [q] = rotmat2quaternion(r)

	q(1) = 0.5*sqrt(1+r(1,1)+r(2,2)+r(3,3));
	q(2) = (r(3,2)-r(2,3)) / 4 / q(1);
	q(3) = (r(1,3)-r(3,1)) / 4 / q(1);
	q(4) = (r(2,1)-r(1,2)) / 4 / q(1);

	n = norm(q);
	q = q./n;