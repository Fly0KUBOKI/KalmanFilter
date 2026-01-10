最終的には、csvの生成をセンサー周期に合わせてcppが読み取りを行う

% 1. 診断スクリプトを実行（コンパイラ情報を確認）
cd kalman
diagnose_mex_binary

% 2. MEXを再ビルド
cd cpp/build
clear mex
build_mex()
clear mex

% 3. バイナリサイズを確認（155KB前後になるべき）
dir ../bin/*.mexw64

% 4. テスト実行
cd ../..
run_simulation(42, true)

初期化期間: 5.0秒 (2000 サンプル)
Start loop
Step 1000 / 20001
Step 2000 / 20001
Step 3000 / 20001
Step 4000 / 20001
Step 5000 / 20001
Step 6000 / 20001
Step 7000 / 20001
Step 8000 / 20001
Step 9000 / 20001
Step 10000 / 20001
Step 11000 / 20001
Step 12000 / 20001
Step 13000 / 20001
Step 14000 / 20001
Step 15000 / 20001
Step 16000 / 20001
Step 17000 / 20001
Step 18000 / 20001
Step 19000 / 20001
Step 20000 / 20001
推定完了

ans =

    3.4156

>> 

kalman\cpp\build\build_mex_log_20260110_224455.txt
kalman\Results\estimation.csv