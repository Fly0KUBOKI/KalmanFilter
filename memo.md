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