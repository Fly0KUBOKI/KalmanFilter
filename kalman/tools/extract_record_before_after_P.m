function extract_record_before_after_P(out_mat, out_txt)
% extract_record_before_after_P  Extract P/K/innovation from record_before/after MAT pairs
% Usage:
%   extract_record_before_after_P()
%   extract_record_before_after_P(out_mat, out_txt)

if nargin<1 || isempty(out_mat)
    results_dir = kalman_tools_utils.get_results_dir(true);
    out_mat = fullfile(results_dir, 'extracted_record_pairs.mat');
end
if nargin<2 || isempty(out_txt)
    results_dir = kalman_tools_utils.get_results_dir(true);
    out_txt = fullfile(results_dir, 'extracted_record_pairs.txt');
end

if ~exist('results_dir', 'var')
    results_dir = kalman_tools_utils.get_results_dir(true);
end
files = dir(fullfile(results_dir,'record_before_*.mat'));
results = struct();
results.rows = {};

for i=1:numel(files)
    before_name = files(i).name;
    % expected format: record_before_<sensor>_<idx>.mat
    tokens = regexp(before_name, '^record_before_(.+)_(\d+)\.mat$','tokens');
    if isempty(tokens), continue; end
    tokens = tokens{1}; sensor = tokens{1}; idx = str2double(tokens{2});
    after_name = sprintf('record_after_%s_%d.mat', sensor, idx);
    before_path = fullfile(results_dir, before_name);
    after_path = fullfile(results_dir, after_name);
    if ~isfile(after_path)
        continue;
    end
    try
        B = load(before_path);
        A = load(after_path);
    catch ME
        warning('Could not load pair %s / %s: %s', before_name, after_name, ME.message);
        continue;
    end
    % pick main variable from loaded struct
    bvars = fieldnames(B); avars = fieldnames(A);
    bmain = kalman_tools_utils.pick_main_var(B, bvars);
    amain = kalman_tools_utils.pick_main_var(A, avars);
    row.sensor = sensor;
    row.idx = idx;
    row.before_file = before_name;
    row.after_file = after_name;
    row.Ppre = kalman_tools_utils.safe_get(bmain, 'P');
    row.Ppost = kalman_tools_utils.safe_get(amain, 'P');
    row.Kpre = kalman_tools_utils.safe_get(bmain, 'K');
    row.Kpost = kalman_tools_utils.safe_get(amain, 'K');
    row.ypre = kalman_tools_utils.safe_get(bmain, 'y');
    row.ypost = kalman_tools_utils.safe_get(amain, 'y');
    results.rows{end+1} = row; %#ok<AGROW>
end

save(out_mat,'results');

% write summary
fid = fopen(out_txt,'w');
if fid<0, warning('Could not open %s for writing', out_txt); return; end
fprintf(fid,'Extracted %d record_before/after pairs\n', numel(results.rows));
for i=1:numel(results.rows)
    r = results.rows{i};
    fprintf(fid,'%d: %s idx=%d files=(%s,%s)\n', i, r.sensor, r.idx, r.before_file, r.after_file);
    if ~isempty(r.Ppre) && ~isempty(r.Ppost)
        D = r.Ppost - r.Ppre;
        fprintf(fid,'  P diff Fro: %.6g, max diag diff: %.6g\n', norm(D,'fro'), max(abs(diag(r.Ppost)-diag(r.Ppre))));
    else
        fprintf(fid,'  Ppre or Ppost missing\n');
    end
end
fclose(fid);
fprintf('Wrote results to %s and %s\n', out_mat, out_txt);
end
