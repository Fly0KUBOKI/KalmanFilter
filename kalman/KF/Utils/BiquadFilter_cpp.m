classdef BiquadFilter_cpp < handle
    % BIQUADFILTER_CPP  Biquad wrapper that uses C++ MEX for 3-element gyro
    % Falls back to MATLAB implementation for scalar/other sizes.

    properties
        b0, b1, b2
        a1, a2
        x1, x2
        y1, y2
        sample_rate
        cutoff_freq
    end

    methods
        function obj = BiquadFilter_cpp(sample_rate, cutoff_freq)
            obj.sample_rate = sample_rate;
            obj.cutoff_freq = cutoff_freq;
            % Initialize scalar states; they may be promoted to vectors
            obj.x1 = 0; obj.x2 = 0; obj.y1 = 0; obj.y2 = 0;
            obj.computeCoefficients();
        end

        function computeCoefficients(obj)
            omega = 2 * pi * obj.cutoff_freq / obj.sample_rate;
            K = tan(omega / 2);
            K2 = K * K;
            Q = 1 / sqrt(2);
            norm = 1 + K / Q + K2;
            obj.b0 = K2 / norm;
            obj.b1 = 2 * obj.b0;
            obj.b2 = obj.b0;
            obj.a1 = 2 * (K2 - 1) / norm;
            obj.a2 = (1 - K / Q + K2) / norm;
        end

        function y = apply(obj, x)
            % If input is a 3-element vector and mex exists, delegate to C++
            % Attempt to call MEX implementation first (unconditional try).
            try
                if numel(x) == 3
                    dt = 1 / obj.sample_rate;
                    out = mex_sensor_filter('gyro', reshape(x,3,1), dt, obj.cutoff_freq);
                    y = out; % returns 3x1 vector
                    return
                end
            catch
                % Fall through to MATLAB fallback on any mex error
            end

            % MATLAB scalar fallback (Direct Form II)
            xv = double(x);
            % If vector input is provided, treat each element as separate channel
            if numel(xv) > 1
                y = zeros(size(xv));
                % Ensure internal states are vectors of same shape
                n = numel(xv);
                if isscalar(obj.x1)
                    obj.x1 = zeros(size(xv));
                    obj.x2 = zeros(size(xv));
                    obj.y1 = zeros(size(xv));
                    obj.y2 = zeros(size(xv));
                else
                    % If existing states have different size, resize preserving tail
                    if numel(obj.x1) ~= n
                        obj.x1 = zeros(size(xv)); obj.x2 = zeros(size(xv)); obj.y1 = zeros(size(xv)); obj.y2 = zeros(size(xv));
                    end
                end

                for k=1:n
                    w = xv(k) - obj.a1 * obj.x1(k) - obj.a2 * obj.x2(k);
                    yk = obj.b0 * w + obj.b1 * obj.x1(k) + obj.b2 * obj.x2(k);
                    obj.x2(k) = obj.x1(k); obj.x1(k) = w;
                    obj.y2(k) = obj.y1(k); obj.y1(k) = yk;
                    y(k) = yk;
                end
            else
                w = xv - obj.a1 * obj.x1 - obj.a2 * obj.x2;
                y = obj.b0 * w + obj.b1 * obj.x1 + obj.b2 * obj.x2;
                obj.x2 = obj.x1; obj.x1 = w;
                obj.y2 = obj.y1; obj.y1 = y;
            end
        end

        function reset(obj)
            obj.x1 = 0; obj.x2 = 0; obj.y1 = 0; obj.y2 = 0;
        end

        function setCutoffFreq(obj, cutoff_freq)
            obj.cutoff_freq = cutoff_freq;
            obj.computeCoefficients();
        end
    end
end
