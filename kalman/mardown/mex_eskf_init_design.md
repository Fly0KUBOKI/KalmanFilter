# mex_eskf_init - Design Notes

目的:
- MATLAB `ESKF` オブジェクトの初期化ロジックを C++ MEX (`mex_eskf_init`) に移植する。
- 期待: MATLAB 側で行っている `state` 構築（`p,v,q,ba,bg,P`）を C++ 側で安全に行い、MATLAB は MEX を呼ぶだけにする。

要件:
- 入力: struct または個別引数で必要な初期値を与える（seed, config, initial_state_fields）
- 出力: C++ 側で確保した `State` 構造体を MATLAB 側で参照するためのハンドル（`uint64` pointer）または MATLAB struct
- 既存コード互換: `ESKF.m` の現在の初期化シーケンスとフィールド名を維持

設計案:
1. シグネチャ
   - `[state_handle] = mex_eskf_init(state_struct, params_struct)`
   - `state_struct` は Matlab `state` 初期値（p,v,q,ba,bg,P）を含む struct（省略フィールドはデフォルトで埋める）
   - `params_struct` は sensor config, noise, gravity などのパラメータ

2. 内部 C++ 型
   - `struct State { double p[3]; double v[3]; double q[4]; double ba[3]; double bg[3]; double P[15*15]; };`
   - メモリは `new` で確保し、`mexMakeMemoryPersistent` を呼ぶ

3. 出力ハンドル
   - MATLAB には `uint64` を返し、以後の `mex_eskf_step` 等はこのハンドルを受け取る
   - `mex_eskf_free(state_handle)` で解放

4. 互換性作業
   - まずは `ESKF.m` の初期化呼び出しを `mex_eskf_init` に差し替え（フォールバックなし）
   - `ESKF` クラス内で `state` を MATLAB struct に変換するラッパーを残す（即時移植中のテスト用）

5. テスト
   - 単体: 小さな `state_struct` を渡し、戻りハンドルを使って `mex_eskf_get_state` で内容を確認
   - 統合: `run_simulation(seed,true)` を MEX-only モードで実行し、結果が既存と一致することを確認

リスクと注意点:
- メモリ管理: MATLAB GC と MEX 持続メモリの扱いに注意する
- struct フィールド順序: MATLAB 側とC++ 側で順序/型が一致することを厳守
- パリティ: 初期化時のノーマライズや P の正定値化処理を MATLAB と一致させる

次の実装ステップ:
1. `mex_eskf_init` のヘッダと基本 MEX 関数を `kalman/cpp/src/mex_eskf_init.cpp` に作成
2. MATLAB `ESKF.m` の初期化呼び出しを `mex_eskf_init` に書き換え（テスト用に並列保持）
3. ビルド: `build_mex({'mex_eskf_init'})` → `run_simulation(42,true)` → 差分検証

