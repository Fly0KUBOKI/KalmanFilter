function fname = generate_and_save_data(params, fname)
% generate_and_save_data - sim_generate を使って模擬データを作り CSV に保存する
% Usage:
%  fname = generate_and_save_data(params)
%  fname = generate_and_save_data(params, 'data.csv')

if nargin<1 || isempty(params)
    params = config_params();
end

% Ensure output settings exist in params
if ~isfield(params, 'output') || ~isfield(params.output, 'dir')
    params.output.dir = fileparts(mfilename('fullpath'));
end

% Call sim_generate which will write truth and sensor CSVs to params.output
sim_generate(params);

% Return the combined filename if requested (deprecated behavior)
if nargout>0
    fname = fullfile(params.output.dir, params.output.truth_filename);
end

end
