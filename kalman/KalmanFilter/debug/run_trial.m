function run_trial(root, base_params, meas_full, maxIter, out_filename, init_from_gps, disable_gating)
% run_trial - run a single debug trial and write CSV diagnostics
% signature matches calls from debug_run.m and analyze_debug_outputs.m

params = base_params;
out_path = fullfile(root, out_filename);
fid = fopen(out_path,'w');
fprintf(fid,'k,t,yg_x,yg_y,d2g,Sg_x,Sg_y,Ppos_x,Ppos_y,x_upd_x,x_upd_y\n');

% detailed debug dump for selected steps (early steps and event around t=8.5)
detailed_out = fullfile(root, ['detailed_' out_filename]);
fid_d = fopen(detailed_out,'w');
fprintf(fid_d, 'k,t,zg_x,zg_y,hg_x,hg_y,y_gx,y_gy,xpred_x,xpred_y,xpred_vx,xpred_vy,xupd_x,xupd_y,xupd_vx,xupd_vy,Ppred_xx,Ppred_yy,Ppred_vxvx,Ppred_vyvy,Sg_x,Sg_y,K_1_1,K_2_2,K_3_1,K_4_2\n');

% initialize state using robust initialization
if init_from_gps
    % Use robust initialization with early measurements
    [x, P] = robust_initialization(meas_full, params);
else
    % Use traditional initialization or fallback to robust method
    if isfield(params.kf,'x0') && ~isempty(params.kf.x0) && ...
       isfield(params.kf,'P0') && ~isempty(params.kf.P0)
        x = params.kf.x0(:);
        P = params.kf.P0;
    else
        % Fallback to robust initialization even for non-GPS case
        [x, P] = robust_initialization(meas_full, params);
    end
end

% Ensure robust initialization is available
if ~exist('robust_initialization', 'file')
    % Fallback to original method if robust_initialization not found
    if init_from_gps && isfield(meas_full,'pos') && ~any(isnan(meas_full.pos(1,:)))
        x = zeros(10,1);
        x(1:2) = meas_full.pos(1,:);
        if isfield(meas_full,'vel') && ~any(isnan(meas_full.vel(1,:)))
            x(3:4) = meas_full.vel(1,:);
        end
    else
        if isfield(params.kf,'x0') && ~isempty(params.kf.x0)
            x = params.kf.x0(:);
        else
            x = [0;0;1;0;0;0;0;0;0;0];
        end
    end
    
    if isfield(params.kf,'P0') && ~isempty(params.kf.P0)
        P = params.kf.P0;
    else
        P = diag([100,100,25,25,10,10,10,10,10,10]); % More conservative initial uncertainty
    end
end

% gating control
if disable_gating
    params.kf.maha_disable_steps = Inf; % effectively disable
    params.kf.gating_enabled = false;
else
    % Enable adaptive gating for better outlier handling
    params.kf.gating_enabled = true;
    params.kf.gate_threshold = 9; % chi-square 95% confidence for 2-DOF
    if ~isfield(params.kf, 'maha_disable_steps')
        params.kf.maha_disable_steps = 5; % allow 5 steps for initialization
    end
end

for k = 1:maxIter
    t = (k-1)*params.dt;
    meas = struct();
    if isfield(meas_full,'pos') && ~any(isnan(meas_full.pos(k,:)))
        meas.gps = meas_full.pos(k,:);
    end
    if isfield(meas_full,'vel') && ~any(isnan(meas_full.vel(k,:)))
        meas.vel = meas_full.vel(k,:);
    end

    params.kf.current_step = k; % let filter know current step
    [x_pred,P_pred,x_upd,P_upd,y,S,K,params] = kf_filter_step(x,P,meas,params);

    % write detailed diagnostics for first few steps and near t=8.5
    dump = false;
    if k <= 3
        dump = true;
    end
    if abs(t - 8.5) < (params.dt/2)
        dump = true;
    end
    if dump
        % measurement z and predicted h for gps if available
        zg_x = NaN; zg_y = NaN; hg_x = NaN; hg_y = NaN; yg_x_d = NaN; yg_y_d = NaN;
        if isfield(meas,'gps') && ~isempty(meas.gps)
            zg_x = meas.gps(1); zg_y = meas.gps(2);
        end
        % try to extract predicted measurement h for gps from last_meas_tags and y
        if isfield(params,'kf') && isfield(params.kf,'last_meas_tags')
            tags = params.kf.last_meas_tags;
            for ii=1:numel(tags)
                if isfield(tags{ii},'name') && strcmp(tags{ii}.name,'gps')
                    rng = tags{ii}.range;
                    if exist('y','var') && ~isempty(y) && exist('h','var') && numel(rng) >= 2 && numel(h) >= max(rng)
                        yg = y(rng);
                        yg_x_d = yg(1); yg_y_d = yg(2);
                        % predicted h values: reconstruct from h if present
                        try
                            hg_x = h(rng(1)); hg_y = h(rng(2));
                        catch
                            hg_x = NaN; hg_y = NaN;
                        end
                    end
                end
            end
        end
        % safe indexing for state elements
        xpred_x = NaN; xpred_y = NaN; xpred_vx = NaN; xpred_vy = NaN;
        xupd_vx = NaN; xupd_vy = NaN;
        if numel(x_pred) >= 4
            xpred_x = x_pred(1); xpred_y = x_pred(2); xpred_vx = x_pred(3); xpred_vy = x_pred(4);
        end
        if numel(x_upd) >= 4
            xupd_vx = x_upd(3); xupd_vy = x_upd(4);
        end
        % P_pred diagonals
        Ppred_xx = NaN; Ppred_yy = NaN; Ppred_vxvx = NaN; Ppred_vyvy = NaN;
        if exist('P_pred','var') && ~isempty(P_pred)
            if size(P_pred,1) >= 4
                Ppred_xx = P_pred(1,1); Ppred_yy = P_pred(2,2); Ppred_vxvx = P_pred(3,3); Ppred_vyvy = P_pred(4,4);
            end
        end
        % S diag for gps (if available)
        Sg_x_d = NaN; Sg_y_d = NaN;
        if exist('S','var') && ~isempty(S) && isfield(params,'kf') && isfield(params.kf,'last_meas_tags')
            tags = params.kf.last_meas_tags;
            for ii=1:numel(tags)
                if isfield(tags{ii},'name') && strcmp(tags{ii}.name,'gps')
                    rng = tags{ii}.range;
                    if numel(rng) >= 2 && size(S,1) >= max(rng)
                        Sg_x_d = S(rng(1),rng(1)); Sg_y_d = S(rng(2),rng(2));
                    end
                end
            end
        end
        % K elements
        K11 = NaN; K22 = NaN; K31 = NaN; K42 = NaN;
        if exist('K','var') && ~isempty(K)
            try
                if size(K,1) >= 1 && size(K,2) >= 1, K11 = K(1,1); end
                if size(K,1) >= 2 && size(K,2) >= 2, K22 = K(2,2); end
                if size(K,1) >= 3 && size(K,2) >= 1, K31 = K(3,1); end
                if size(K,1) >= 4 && size(K,2) >= 2, K42 = K(4,2); end
            catch
            end
        end
        % Build values vector and dynamic format to avoid mismatch
        xupd1 = NaN; xupd2 = NaN;
        if numel(x_upd) >= 2
            xupd1 = x_upd(1); xupd2 = x_upd(2);
        end
        vals = [k-1, t, zg_x, zg_y, hg_x, hg_y, yg_x_d, yg_y_d, xpred_x, xpred_y, xpred_vx, xpred_vy, xupd1, xupd2, xupd_vx, xupd_vy, Ppred_xx, Ppred_yy, Ppred_vxvx, Ppred_vyvy, Sg_x_d, Sg_y_d, K11, K22, K31, K42];
        fmt = [repmat('%.6g,',1,numel(vals)-1) '%.6g\n'];
        fprintf(fid_d, fmt, vals);
    end

    % extract gps diagnostics
    yg_x = NaN; yg_y = NaN; d2g = NaN; Sg_x = NaN; Sg_y = NaN; Ppos_x = NaN; Ppos_y = NaN;
    if ~isempty(y) && isfield(params,'kf') && isfield(params.kf,'last_meas_tags')
        tags = params.kf.last_meas_tags;
        for ii=1:numel(tags)
            if isfield(tags{ii},'name') && strcmp(tags{ii}.name,'gps')
                rng = tags{ii}.range;
                if numel(rng) >= 2
                    yg = y(rng);
                    Sg = S(rng,rng);
                    yg_x = yg(1); yg_y = yg(2);
                    % robust d2 computation
                    if all(isfinite(Sg(:))) && det(Sg) ~= 0
                        d2g = (yg' / Sg) * yg;
                    else
                        d2g = NaN;
                    end
                    Sg_x = Sg(1,1); Sg_y = Sg(2,2);
                    Ppos_x = P_pred(1,1); Ppos_y = P_pred(2,2);
                end
            end
        end
    end

    if numel(x_upd) >= 2
        fprintf(fid,'%d,%.6f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g\n', k-1, t, yg_x, yg_y, d2g, Sg_x, Sg_y, Ppos_x, Ppos_y, x_upd(1), x_upd(2));
    else
        fprintf(fid,'%d,%.6f,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,NaN,NaN\n', k-1, t, yg_x, yg_y, d2g, Sg_x, Sg_y, Ppos_x, Ppos_y);
    end

    x = x_upd; P = P_upd;
end
fclose(fid);
fprintf('Wrote %s\n', out_path);
end


