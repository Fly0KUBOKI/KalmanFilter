classdef KalmanCompute
    % KalmanCompute - MEX-only thin wrapper. MATLAB fallback implementations removed.
    properties (Constant)
        MEX_FUNCTION = 'mex_kalman_compute';
    end

    methods (Static)
        function assert_mex()
            if exist(KalmanCompute.MEX_FUNCTION, 'file') ~= 3
                error('KalmanCompute:missing_mex', 'mex_kalman_compute not found. MATLAB implementations removed; build the C++ MEX.');
            end
        end

        function varargout = call(func, varargin)
            KalmanCompute.assert_mex();
            [varargout{1:nargout}] = mex_kalman_compute(func, varargin{:});
        end

        function q_out = quat_multiply(q1, q2)
            q_out = KalmanCompute.call('quat_multiply', single([q1(:); q2(:)]));
        end

        function q_out = quat_normalize(q)
            q_out = KalmanCompute.call('quat_normalize', single(q(:)));
        end

        function q_out = quat_conjugate(q)
            q_out = KalmanCompute.call('quat_conjugate', single(q(:)));
        end

        function q_out = quat_inverse(q)
            q_out = KalmanCompute.call('quat_inverse', single(q(:)));
        end

        function R = quat_to_rotation_matrix(q)
            R = KalmanCompute.call('quat_to_rotation_matrix', single(q(:)));
        end

        function euler = quat_to_euler(q)
            euler = KalmanCompute.call('quat_to_euler', single(q(:)));
        end

        function q = quat_from_euler(euler)
            q = KalmanCompute.call('quat_from_euler', single(euler(:)));
        end

        function q = quat_from_small_angle(theta)
            q = KalmanCompute.call('quat_from_small_angle', single(theta(:)));
        end

        function q_new = quat_integrate(q, omega, dt)
            q_new = KalmanCompute.call('quat_integrate', single([q(:); omega(:); dt]));
        end

        function skew = rot_skew_symmetric(v)
            skew = KalmanCompute.call('rot_skew_symmetric', single(v(:)));
        end

        function R = rot_from_euler(euler)
            R = KalmanCompute.call('rot_from_euler', single(euler(:)));
        end

        function euler = rot_to_euler(R)
            euler = KalmanCompute.call('rot_to_euler', single(R(:)));
        end

        function R_ortho = rot_orthonormalize(R)
            R_ortho = KalmanCompute.call('rot_orthonormalize', single(R(:)));
        end

        function v_rot = rot_apply_rotation(R, v)
            v_rot = KalmanCompute.call('rot_apply_rotation', single([R(:); v(:)]));
        end
    end
end
