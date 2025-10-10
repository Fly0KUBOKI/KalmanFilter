function varargout = eskf_utils(action, varargin)
% ESKF utility functions (single-file multiplexer)
% Usage:
%   R = eskf_utils('quat_to_rotm', q)
%   q = eskf_utils('quatnormalize', q)
%   q = eskf_utils('quatmultiply', q1, q2)
%   S = eskf_utils('skew', v)
%
switch lower(action)
    case 'quat_to_rotm'
        q = varargin{1}; q = q(:); q = eskf_utils('quatnormalize', q);
        qw=q(1); qx=q(2); qy=q(3); qz=q(4);
        R = [1-2*(qy^2+qz^2), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw);
             2*(qx*qy+qz*qw),   1-2*(qx^2+qz^2), 2*(qy*qz-qx*qw);
             2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx^2+qy^2)];
        varargout{1} = R;
    case 'quatnormalize'
        q = varargin{1}; q = q(:);
        n = norm(q);
        if n==0
            q = [1;0;0;0];
        else
            q = q / n;
        end
        varargout{1} = q;
    case 'quatmultiply'
        a = varargin{1}(:); b = varargin{2}(:);
        % Hamilton product: q = a * b
        aw=a(1); av=a(2:4); bw=b(1); bv=b(2:4);
        s = aw*bw - av'*bv;
        v = aw*bv + bw*av + cross(av,bv);
        varargout{1} = [s; v];
    case 'small_angle_quat'
        % convert small rotation vector theta (3x1) to quaternion approx
        th = varargin{1}(:);
        th2 = th'*th;
        if th2 < 1e-8
            q = [1; 0.5*th];
        else
            angle = sqrt(th2);
            axis = th / angle;
            q = [cos(angle/2); axis*sin(angle/2)];
        end
        varargout{1} = eskf_utils('quatnormalize', q);
    case 'skew'
        v = varargin{1}(:);
        S = [  0   -v(3)  v(2);
              v(3)   0   -v(1);
             -v(2) v(1)    0 ];
        varargout{1} = S;
    otherwise
        error('Unknown eskf_utils action: %s', action);
end
end
