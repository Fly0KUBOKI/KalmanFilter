function varargout = quat_lib(action, varargin)
    % QUAT_LIB  Quaternion and small-angle utilities (copied from eskf_utils)
    % Usage:
    %   R = quat_lib('quat_to_rotm', q)
    %   q = quat_lib('quatnormalize', q)
    %   q = quat_lib('quatmultiply', q1, q2)
    %   S = quat_lib('skew', v)

    switch lower(action)
        case 'quat_to_rotm'
            q = varargin{1}; q = q(:); q = quat_lib('quatnormalize', q);
            qw=q(1); qx=q(2); qy=q(3); qz=q(4);
            R = [1-2*(qy^2+qz^2), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw);
                2*(qx*qy+qz*qw),   1-2*(qx^2+qz^2), 2*(qy*qz-qx*qw);
                2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx^2+qy^2)];
            varargout{1} = R;
        case 'quatnormalize'
            q = varargin{1}; q = q(:);
            % defensive: handle non-finite inputs and near-zero norm
            if any(~isfinite(q))
                warning('quatnormalize: non-finite quaternion input; resetting to identity [1;0;0;0]');
                q = [1;0;0;0];
            else
                n = norm(q);
                if n < eps
                    q = [1;0;0;0];
                else
                    q = q / n;
                end
            end
            varargout{1} = q;
        case 'quatmultiply'
            a = varargin{1}(:); b = varargin{2}(:);
            % validate inputs
            if numel(a) ~= 4 || numel(b) ~= 4
                error('quatmultiply: inputs must be 4-element quaternions');
            end
            if any(~isfinite(a)) || any(~isfinite(b))
                error('quatmultiply: non-finite quaternion input detected. a=%s b=%s', mat2str(a'), mat2str(b'));
            end
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
            varargout{1} = quat_lib('quatnormalize', q);
        case 'skew'
            v = varargin{1}(:);
            S = [  0   -v(3)  v(2);
                v(3)   0   -v(1);
                -v(2) v(1)    0 ];
            varargout{1} = S;
        otherwise
            error('Unknown quat_lib action: %s', action);
    end
end
