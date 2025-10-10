function att_avg = calculate_attitude_average(att_buffer)
% 姿勢推定結果の平均を計算
att_sum = zeros(3, 1, 'single');
count = 0;
for i = 1:length(att_buffer)
    if ~isempty(att_buffer{i})
        att_sum = att_sum + att_buffer{i}.attitude;
        count = count + 1;
    end
end
if count > 0
    att_avg = struct();
    att_avg.attitude = att_sum / count;
else
    att_avg = [];
end
end
