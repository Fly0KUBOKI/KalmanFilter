% record_and_analyze.m
% マイクから音声を録音して解析する簡易スクリプト
% 使い方:
%   record_and_analyze()        % デフォルト 5 秒で録音
%   record_and_analyze(10)      % 10 秒録音
function record_and_analyze(duration)
if nargin < 1 || isempty(duration)
    duration = 5; % 秒
end

fs = 44100; % サンプリング周波数(Hz)
fprintf('録音を開始します: %d 秒, Fs=%d Hz\n', duration, fs);

% シンプルに audiorecorder を使用（Audio Toolbox が無くても動作）
rec = audiorecorder(fs, 16, 1);
recordblocking(rec, duration);
fprintf('録音終了。データを取得中...\n');

y = getaudiodata(rec);

% 正規化（振幅が大きすぎる場合）
if max(abs(y)) > 0
    y = y / max(abs(y));
end

% WAV に保存
outFile = fullfile(pwd, 'recording.wav');
try
    audiowrite(outFile, y, fs);
    fprintf('録音ファイルを保存しました: %s\n', outFile);
catch
    warning('録音ファイルの保存に失敗しました。audiowrite が使用できない可能性があります。');
end

% --- 解析: 波形, 振幅スペクトル, スペクトログラム ---
t = (0:length(y)-1) / fs;
figure('Name','録音解析','NumberTitle','off');
subplot(3,1,1);
plot(t, y, 'k');
xlabel('時間 [s]');
ylabel('振幅');
title('波形');
grid on

% FFT
n = length(y);
Y = fft(y);
f = (0:n-1) * (fs/n);
mag = abs(Y)/n;
subplot(3,1,2);
plot(f(1:floor(n/2)), mag(1:floor(n/2)), 'b');
xlabel('周波数 [Hz]');
ylabel('振幅');
title('振幅スペクトル');
xlim([0 fs/2]);
grid on

% スペクトログラム
subplot(3,1,3);
win = round(0.03*fs); % 30 ms
noverlap = round(0.02*fs);
nfft = max(256, 2^nextpow2(win));
sgram = spectrogram(y, win, noverlap, nfft, fs, 'yaxis');
imagesc((0:size(sgram,2)-1)*(win-noverlap)/fs, linspace(0,fs/2,size(sgram,1)), 20*log10(abs(sgram(1:floor(end/2),:))+eps));
axis xy
xlabel('時間 [s]');
ylabel('周波数 [Hz]');
title('スペクトログラム (dB)');
colorbar;

% --- 基本周波数推定（自相関法と FFT 最大ピーク） ---
f0_ac = estimate_pitch_autocorr(y, fs);
f0_fft = estimate_dominant_freq_fft(y, fs);

fprintf('推定基本周波数 (自相関法): %.1f Hz\n', f0_ac);
fprintf('推定支配周波数 (FFTピーク): %.1f Hz\n', f0_fft);

end

function f0 = estimate_pitch_autocorr(x, fs)
% 単純な自相関に基づく基本周波数推定
% 入力はモノラル信号 x, サンプリング周波数 fs
x = x(:);
L = length(x);
if L < fs*0.03
    f0 = NaN; return
end

% 先頭の 30 ms を使う（短い音声に対応）
winlen = min(L, round(0.03*fs));
w = x(1:winlen) .* hamming(winlen);

acor = xcorr(w);
mid = ceil(length(acor)/2);
acor = acor(mid:end);

% 基本周波数は通常 50Hz ~ 500Hz の範囲に入ると想定
minF = 50; maxF = 500;
minLag = floor(fs/maxF);
maxLag = ceil(fs/minF);
search = acor(minLag+1:min(maxLag+1, length(acor)));
if isempty(search)
    f0 = NaN; return
end
[pks, locs] = findpeaks(search);
if isempty(pks)
    f0 = NaN; return
end
[~, idx] = max(pks);
lag0 = locs(idx) + minLag - 1;
f0 = fs / lag0;
end

function fdom = estimate_dominant_freq_fft(x, fs)
x = x(:);
N = length(x);
if N < 2
    fdom = NaN; return
end
Y = abs(fft(x.*hamming(N))); Y = Y(1:floor(N/2));
f = (0:floor(N/2)-1) * (fs/N);
[~, idx] = max(Y);
fdom = f(idx);
end
