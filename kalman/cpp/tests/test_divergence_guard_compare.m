% Test comparison: mex_sensor_filter('divergence_regularize', P) vs MATLAB DivergenceGuard.regularize_covariance
results_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'Results');
if ~exist(results_dir, 'dir'), mkdir(results_dir); end

% Ensure MATLAB-side utilities are on the path (DivergenceGuard etc.)
if isempty(mfilename('fullpath'))
    script_dir = pwd;
else
    script_path = mfilename('fullpath');
    script_dir = fileparts(script_path);
end
repo_root = fullfile(script_dir, '..', '..');
try
    repo_root = char(java.io.File(repo_root).getCanonicalPath());
catch
    repo_root = repo_root; % fallback
end
addpath(fullfile(repo_root, 'KF', 'Utils'));
out_csv = fullfile(results_dir, 'divergence_compare_results.csv');
out_mat = fullfile(results_dir, 'divergence_compare_pairs.mat');

tests = {};
% 1: tiny diag
tests{end+1} = struct('desc','tiny_diag','P', eye(15)*1e-12);
% 2: varied diag
d = [1e-12,1e-9,1e-6,1e-3,1,10,100,1e-5,1e-4,1e-2,0.5,2,5,1e-8,1e-7]; tests{end+1} = struct('desc','varied_diag','P', diag(d));
% 3: small random SPD
A = randn(15); tests{end+1} = struct('desc','rand_spd_small','P', (A'*A)*1e-6);
% 4: near singular
[U,~,~] = svd(randn(15)); svec = 1+rand(15,1); svec(end) = 1e-16; tests{end+1} = struct('desc','near_singular','P', U*diag(svec)*U');
% 5: small off-diagonals
P5 = eye(15); P5(1,2)=1e-16; P5(2,1)=1e-16; tests{end+1} = struct('desc','off_diag_small','P', P5);
% 6: large block-scaled
vals = [1e6*ones(3,1);1e4*ones(3,1);100*ones(3,1);1e2*ones(3,1);1e-1*ones(3,1)]; tests{end+1} = struct('desc','large_block','P', diag(vals));

% Prepare output
fid = fopen(out_csv,'w');
fprintf(fid, 'id,desc,fro_diff,max_abs_diff,max_diag_diff,rcond_mex,rcond_mat\n');
fclose(fid);

addpath(fullfile(fileparts(mfilename('fullpath')),'..','bin'));

pairs = struct('desc',{},'P_mex',{},'P_mat',{});

for i=1:numel(tests)
    T = tests{i};
    P = (T.P + T.P')/2;

    % run mex regularize
    try
        P_mex = mex_sensor_filter('divergence_regularize', P);
    catch ME
        warning('mex regularize failed: %s', ME.message);
        P_mex = NaN(size(P));
    end

    % force MATLAB fallback
    setenv('FORCE_MATLAB_FILTERS','1');
    dg = DivergenceGuard();
    try
        P_mat = dg.regularize_covariance(P);
    catch ME
        warning('mat regularize failed: %s', ME.message);
        P_mat = NaN(size(P));
    end
    setenv('FORCE_MATLAB_FILTERS','');

    % metrics
    fro_diff = norm(P_mex - P_mat, 'fro');
    max_abs = max(abs(P_mex(:) - P_mat(:)));
    max_diag = max(abs(diag(P_mex) - diag(P_mat)));
    r_mex = rcond(P_mex);
    r_mat = rcond(P_mat);

    fid = fopen(out_csv,'a');
    fprintf(fid, '%d,%s,%.12g,%.12g,%.12g,%.12g,%.12g\n', i, T.desc, fro_diff, max_abs, max_diag, r_mex, r_mat);
    fclose(fid);

    pairs(i).desc = T.desc; pairs(i).P_mex = P_mex; pairs(i).P_mat = P_mat;
    fprintf('Test %d (%s): fro_diff=%.12g, max_diag=%.12g\n', i, T.desc, fro_diff, max_diag);
end

save(out_mat,'pairs');
fprintf('Wrote results to %s and %s\n', out_csv, out_mat);
