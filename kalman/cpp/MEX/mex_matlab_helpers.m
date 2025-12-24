function varargout = mex_matlab_helpers(cmd, varargin)
% MEX_MATLAB_HELPERS  Compatibility helper (M-file fallback for mex)
% Usage:
%  data = mex_matlab_helpers('get_field', obs, field_names_cell, idx, num_cols)
%  tf   = mex_matlab_helpers('has_field', obs, field_names_cell)
%  eul  = mex_matlab_helpers('get_euler', q)

if nargin < 1
    error('mex_matlab_helpers:input', 'Command required');
end

cmd = lower(cmd);
switch cmd
    case 'get_field'
        obs = varargin{1};
        field_names = varargin{2};
        if nargin >= 4 && ~isempty(varargin{3}), idx = varargin{3}; else idx = [] ; end
        if nargin >= 5 && ~isempty(varargin{4}), num_cols = varargin{4}; else num_cols = [] ; end

        for i = 1:length(field_names)
            fname = field_names{i};
            if isfield(obs, fname)
                v = obs.(fname);
                % Ensure numeric
                if ~isnumeric(v)
                    try v = double(v); catch, end
                end
                % Handle column vectors / row vectors and matrices
                if isempty(idx)
                    data = v;
                else
                    try
                        data = v(idx, :);
                    catch
                        % try linear indexing
                        data = v(idx);
                    end
                end
                if ~isempty(num_cols)
                    % Ensure result has num_cols columns
                    if isvector(data)
                        data = reshape(data, [], num_cols);
                    else
                        % try to trim or pad
                        [r,c] = size(data);
                        if c > num_cols
                            data = data(:,1:num_cols);
                        elseif c < num_cols
                            data = [data, zeros(r, num_cols-c)];
                        end
                    end
                end
                varargout{1} = data;
                return;
            end
        end
        error('mex_matlab_helpers:get_field', 'None of the fields found: %s', strjoin(field_names, ', '));

    case 'has_field'
        obs = varargin{1};
        field_names = varargin{2};
        tf = false;
        for i = 1:length(field_names)
            if isfield(obs, field_names{i})
                tf = true; break;
            end
        end
        varargout{1} = tf;

    case 'get_euler'
        q = varargin{1};
        % Prefer existing quaternion MEX if available
        if exist('mex_quaternion_lib','file') == 3 || exist('mex_quaternion_lib','file') == 2
            try
                varargout{1} = mex_quaternion_lib('to_euler', q);
                return;
            catch
            end
        end
        % Fallback: compute simple conversion (deg)
        try
            % q is expected as [w;x;y;z]
            w = q(1); x = q(2); y = q(3); z = q(4);
            % roll (phi), pitch (theta), yaw (psi)
            sinr_cosp = 2*(w*x + y*z);
            cosr_cosp = 1 - 2*(x^2 + y^2);
            phi = atan2(sinr_cosp, cosr_cosp);

            sinp = 2*(w*y - z*x);
            if abs(sinp) >= 1
                theta = sign(sinp) * (pi/2);
            else
                theta = asin(sinp);
            end

            siny_cosp = 2*(w*z + x*y);
            cosy_cosp = 1 - 2*(y^2 + z^2);
            psi = atan2(siny_cosp, cosy_cosp);

            varargout{1} = rad2deg([phi; theta; psi]);
        catch
            varargout{1} = [0;0;0];
        end

    otherwise
        error('mex_matlab_helpers:unknown', 'Unknown command: %s', cmd);
end

end
