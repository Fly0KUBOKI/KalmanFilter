function [z, h, H, R, meas_tags] = assemble_measurements(meas, x_pred, params)
%ASSEMBLE_MEASUREMENTS Build measurement vector/matrices from meas struct
%   [z,h,H,R,meas_tags] = assemble_measurements(meas, x_pred, params)
%   This function is ASCII-safe and concatenates per-sensor measurement
%   blocks returned by assemble_measurement_blocks.

% If caller already provided preassembled measurement fields, use them.
if isfield(meas, 'z') && isfield(meas, 'H') && isfield(meas, 'R')
    % Caller provided preassembled measurement fields (use valid names)
    z = getfield(meas, 'z');
    h = getfield(meas, 'h');
    H = getfield(meas, 'H');
    R = getfield(meas, 'R');
    if isfield(meas, 'meas_tags')
        meas_tags = getfield(meas, 'meas_tags');
    else
        meas_tags = {};
    end
    return;
end

% Get per-sensor blocks and concatenate. Each block is a struct with fields
%: name, z, h, H, R
blocks = assemble_measurement_blocks(meas, x_pred, params);

z = [];
h = [];
H = zeros(0, numel(x_pred));
R = [];
meas_tags = {};

start_idx = 1;
for iBlock = 1:numel(blocks)
    b = blocks{iBlock};

    if isempty(b) || ~isfield(b, 'z')
        continue;
    end

    z = [z; b.z(:)];
    h = [h; b.h(:)];
    H = [H; b.H];
    R = blkdiag(R, b.R);

    idx_range = start_idx:(start_idx + numel(b.z) - 1);
    meas_tags{end+1} = struct('name', b.name, 'range', idx_range);
    start_idx = start_idx + numel(b.z);
end

end
