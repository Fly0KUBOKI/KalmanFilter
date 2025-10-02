function data = get_col_from_table(T, names)
% get_col_from_table - extract columns from table T by list of possible names
%   data = get_col_from_table(T, names)
%   names : cell array of candidate column names (e.g. {'accel3_x','accel3_y'})
% Returns NxD matrix where missing columns are NaN.

N = height(T);
D = numel(names);
data = NaN(N,D);
for j=1:D
    nm = names{j};
    if ismember(nm, T.Properties.VariableNames)
        data(:,j) = T.(nm);
    else
        % try common alternates
        alt1 = ['meas_' nm];
        alt2 = [nm '_meas'];
        if ismember(alt1, T.Properties.VariableNames)
            data(:,j) = T.(alt1);
        elseif ismember(alt2, T.Properties.VariableNames)
            data(:,j) = T.(alt2);
        else
            % also try dropping suffix/prefix variations: x/y/z naming
            % e.g., if nm ends with _x, try same base without axis
            underscoreIdx = strfind(nm, '_');
            if ~isempty(underscoreIdx)
                base = nm(1:underscoreIdx(end)-1);
                if ismember(base, T.Properties.VariableNames)
                    data(:,j) = T.(base);
                end
            end
        end
    end
end
end
