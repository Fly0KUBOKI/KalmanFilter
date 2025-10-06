function plot_csv_variables()
% plot_csv_variables - CSV の全変数を一括表示します
% Usage:
%   plot_csv_variables('sim_data.csv')
%   plot_csv_variables()
%
% 動作:
% - テーブルとして CSV を読み込みます。
% - 変数名を解析し、末尾が _x/_y/_z の列はベース名でグループ化して
%   同一グラフにプロットします（例: accel3_x, accel3_y, accel3_z）。
% - グループごとに別の figure ウィンドウを作成します。
% - 時系列軸として列 't' があればそれを x 軸に使い、なければ行インデックスを使用します.

% No command-line args: always use sim_data.csv by default; if not found ask user
fname = fullfile(pwd,'sim_data.csv');
if ~exist(fname,'file')
    [f,p] = uigetfile({'*.csv','CSV files'}, 'Select CSV file to plot');
    if isequal(f,0)
        disp('No file selected.');
        return;
    end
    fname = fullfile(p,f);
end

% read table first to know variables; then ask user which group(s) to show
T = readtable(fname);
vars = T.Properties.VariableNames;

% Decide x axis
if any(strcmp(vars,'t'))
    t = T.t;
else
    t = (1:height(T))';
end

% Group variables by base name: strip suffix _x/_y/_z if present
groups = containers.Map();
colToBase = containers.Map();
for i = 1:numel(vars)
    v = vars{i};
    % skip time column
    if strcmp(v,'t'), continue; end
    % detect _x, _y, _z suffix pattern
    tok = regexp(v,'^(.*)_(x|y|z)$','tokens','once');
    if ~isempty(tok)
        base = tok{1};
    else
        base = v;
    end
    if isKey(groups, base)
        tmp = groups(base);
        tmp{end+1} = v; %#ok<AGROW>
        groups(base) = tmp;
    else
        groups(base) = {v};
    end
    colToBase(v) = base;
end

keysG = keys(groups);
colors = lines(4);

% Ask user which groups to display via list dialog
dlgList = keysG;
[sel, ok] = listdlg('PromptString','Select groups to plot (Cancel = all):','ListString',dlgList,'ListSize',[300 400]);
if ok == 0
    % user cancelled -> plot all
    sel = 1:numel(dlgList);
end

keysG = dlgList(sel);

for k = 1:numel(keysG)
    name = keysG{k};
    cols = groups(name);
    % Create a new figure per group
    fig = figure('Name', sprintf('CSV: %s', name), 'NumberTitle','off');
    ax = axes(fig);
    hold(ax,'on'); grid(ax,'on');
    title(ax, name, 'Interpreter','none');
    xlabel(ax, 't');
    ylabel(ax, 'value');

    legendEntries = {};
    for j = 1:numel(cols)
        col = cols{j};
        y = T.(col);
        % if y is a 2/3-column vector stored in a single table variable, try to handle
        if ismatrix(y) && size(y,2)>1 && size(y,1)==height(T)
            % multiple columns inside one table variable, plot each column
            for c = 1:size(y,2)
                plot(ax, t, y(:,c), 'Color', colors(mod(c-1,size(colors,1))+1,:), 'LineWidth',1);
                legendEntries{end+1} = sprintf('%s[%d]', col, c); %#ok<AGROW>
            end
        else
            plot(ax, t, y, 'LineWidth',1);
            legendEntries{end+1} = col; %#ok<AGROW>
        end
    end
    legend(ax, legendEntries, 'Interpreter','none');
    hold(ax,'off');
end

end
