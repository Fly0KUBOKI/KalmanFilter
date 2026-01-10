# 環境依存問題の根本原因分析と修正計画 v2

**作成日**: 2026-01-10  
**問題**: 2台のPCで同じソースコードから異なる動作のMEXバイナリが生成される

---

## 🔴 問題の症状

| 項目 | このPC（正常） | ノートPC（異常） | 差異 |
|------|--------------|----------------|------|
| mex_run_eskf.mexw64 | 155,648 bytes | 980,480 bytes | **6.3倍** |
| mex_meukf_step_v2.mexw64 | 11,264 bytes | 894,464 bytes | **79倍** |
| ジャイロバイアス | 更新される | **常にゼロ** | 完全に異なる |
| Position RMSE | ~0.32m | 64,000m+ | 異常に大きい |
| 処理時間 | ~1.1秒/run | ~2.9秒/run | 2.6倍遅い |

---

## 🔍 根本原因分析

### 仮説1: コンパイラの違い（確度: 高）

**証拠**:
- バイナリサイズが6.3倍〜79倍違う
- 小さいバイナリ（155KB）= Release/最適化ビルド
- 大きいバイナリ（980KB）= Debug/未最適化ビルド

**問題**:
MATLABの`mex`コマンドは、システムにインストールされているコンパイラを自動選択する。
2台のPCで異なるコンパイラ（またはバージョン）が選択されている可能性が高い。

**確認方法**:
```matlab
mex.getCompilerConfigurations('C++', 'Selected')
```

### 仮説2: コンパイラフラグの違い（確度: 高）

**現在のbuild_mex.mの問題点**:

```matlab
% 現在のコード（build_mex.m 51-59行）
if ispc
    compile_opts = [compile_opts, {'-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'}];
    old_compflags = getenv('COMPFLAGS');
    setenv('COMPFLAGS', '/utf-8 /fp:precise /arch:SSE2');  % ← MSVCフラグ
else
    compile_opts = [compile_opts, {'-O2', '-msse2', '-fno-fast-math', '-ffloat-store'}];
end
```

**問題**:
1. `setenv('COMPFLAGS', ...)` はMSVC用のフラグ。MinGW環境では無視される
2. 最適化フラグ（`-O2`など）がWindowsでは渡されていない
3. コンパイラによってデフォルトの最適化レベルが異なる

### 仮説3: OneDrive同期の問題（確度: 中）

**証拠**:
- 両PCのログタイムスタンプが異なる（22:22 vs 22:28）
- バイナリが同期される前に実行された可能性

**問題**:
OneDriveはビルド後に`.mexw64`ファイルを同期するが、MEXファイルがロード中の場合は同期がスキップされる可能性がある。

### 仮説4: 未定義動作（確度: 中〜高）

**コードレビューで発見した問題点**:

#### 問題A: 静的関数ポインタのコンパイラ依存

```cpp
// meukf_update.cpp 22-40行
static ukf::UKFUpdate<15,3,float>::VectorM h_func_gps_fn(const ukf::UKFUpdate<15,3,float>::VectorN& xv) {
    ukf::UKFUpdate<15,3,float>::VectorM zv;
    zv(0,0) = xv(0,0);
    zv(1,0) = xv(1,0);
    zv(2,0) = xv(2,0);
    return zv;
}
```

問題：`static`関数のアドレスがファイル単位で異なる可能性。テンプレート特殊化と組み合わせると未定義動作の原因になりうる。

#### 問題B: テンプレートの暗黙的実体化

```cpp
// ukf_update.hpp 60行
template<typename ObsFunc>
static bool update(
    VectorN& x,
    MatrixNN& P,
    const VectorM& z,
    ObsFunc h_func,  // ← 関数オブジェクトの型がコンパイラ依存
    ...
)
```

問題：ラムダ式や関数ポインタを`ObsFunc`として渡すと、コンパイラによって異なるコード生成が行われる。

#### 問題C: 行列のメモリレイアウト

```cpp
// mex_run_eskf_impl.hpp 217-220行
float P_tmp[15*15];
mex_conv::mxArrayToFloatArray(f_P, P_tmp, 15*15);
for (int r=0;r<15;++r) for (int c=0;c<15;++c) input.prev_state.P[r*15 + c] = P_tmp[c*15 + r];
```

問題：MATLAB column-major → C++ row-major変換。ループ順序やメモリアクセスパターンがコンパイラ最適化に影響。

---

## 📋 修正計画（優先順位順）

### Phase 0: 緊急診断（即時実行）

両PCで以下を実行して環境差を特定：

```matlab
cd kalman
diagnose_mex_binary  % ← 新規作成した診断スクリプト
```

**収集すべき情報**:
1. コンパイラ名とバージョン（`mex.getCompilerConfigurations`）
2. 最適化レベル
3. 5ステップ後のジャイロバイアス値

### Phase 1: ビルドシステムの統一（1日）

**目標**: 全PCで同一のバイナリを生成

#### 1.1 コンパイラの明示的指定

```matlab
% build_mex.m の修正案
function build_mex(targets)
    % コンパイラを明示的に設定
    cc = mex.getCompilerConfigurations('C++', 'Installed');
    
    % MSVCを優先、なければMinGW
    msvc_idx = find(contains({cc.Name}, 'Microsoft'), 1, 'first');
    mingw_idx = find(contains({cc.Name}, 'MinGW'), 1, 'first');
    
    if ~isempty(msvc_idx)
        mex.setCompilerConfigurations(cc(msvc_idx));
        fprintf('Using MSVC: %s\n', cc(msvc_idx).Name);
    elseif ~isempty(mingw_idx)
        mex.setCompilerConfigurations(cc(mingw_idx));
        fprintf('Using MinGW: %s\n', cc(mingw_idx).Name);
    else
        error('No supported C++ compiler found');
    end
    
    % ... 残りのビルドロジック
end
```

#### 1.2 最適化フラグの明示的指定（MSVC/MinGW両対応）

```matlab
% コンパイラ検出
cc = mex.getCompilerConfigurations('C++', 'Selected');

if contains(cc.Name, 'Microsoft')
    % MSVC
    compile_opts = {
        'CXXFLAGS=$CXXFLAGS /O2',     % 最適化レベル2
        'CXXFLAGS=$CXXFLAGS /fp:precise',
        'CXXFLAGS=$CXXFLAGS /arch:SSE2',
        'CXXFLAGS=$CXXFLAGS /DNDEBUG',
        '-DWIN32', '-D_CRT_SECURE_NO_WARNINGS'
    };
elseif contains(cc.Name, 'MinGW')
    % MinGW
    compile_opts = {
        'CXXFLAGS=$CXXFLAGS -O2',
        'CXXFLAGS=$CXXFLAGS -msse2',
        'CXXFLAGS=$CXXFLAGS -fno-fast-math',
        'CXXFLAGS=$CXXFLAGS -DNDEBUG',
        '-DWIN32'
    };
else
    error('Unsupported compiler: %s', cc.Name);
end
```

#### 1.3 バイナリサイズの検証

```matlab
% ビルド後に自動検証
mex_path = fullfile(bin_dir, 'mex_run_eskf.mexw64');
info = dir(mex_path);

% 期待サイズ範囲（150KB〜200KB）
if info.bytes > 500000
    warning('Binary too large (%d bytes) - may contain debug symbols', info.bytes);
elseif info.bytes < 50000
    warning('Binary too small (%d bytes) - may be stripped too much', info.bytes);
else
    fprintf('Binary size OK: %d bytes\n', info.bytes);
end
```

### Phase 2: 未定義動作の除去（2日）

#### 2.1 静的関数ポインタの廃止

```cpp
// 現在（meukf_update.cpp）
static ukf::UKFUpdate<15,3,float>::VectorM h_func_gps_fn(const ukf::UKFUpdate<15,3,float>::VectorN& xv) { ... }

// 修正後: ファンクタクラスを使用
struct GPSObservationModel {
    using VectorN = ukf::UKFUpdate<15,3,float>::VectorN;
    using VectorM = ukf::UKFUpdate<15,3,float>::VectorM;
    
    VectorM operator()(const VectorN& xv) const {
        VectorM zv;
        zv(0,0) = xv(0,0);
        zv(1,0) = xv(1,0);
        zv(2,0) = xv(2,0);
        return zv;
    }
};
```

#### 2.2 UKFUpdate テンプレートの明示的実体化

```cpp
// ukf_update.cpp（新規作成）
#include "ukf_update.hpp"

// 明示的実体化（バイナリ互換性を保証）
template class ukf::UKFUpdate<15, 3, float>;
template class ukf::UKFUpdate<15, 1, float>;
template class ukf::UKFUpdate<15, 2, float>;
```

#### 2.3 行列変換の一貫性確保

```cpp
// mex_type_conversion.hpp に統一関数を追加
namespace mex_conv {

// MATLAB column-major → C++ row-major
template<int N, int M>
void matlab_to_cpp_matrix(const float* matlab_data, float* cpp_data) {
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < M; ++c) {
            cpp_data[r * M + c] = matlab_data[c * N + r];
        }
    }
}

// C++ row-major → MATLAB column-major
template<int N, int M>
void cpp_to_matlab_matrix(const float* cpp_data, float* matlab_data) {
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < M; ++c) {
            matlab_data[c * N + r] = cpp_data[r * M + c];
        }
    }
}

} // namespace mex_conv
```

### Phase 3: OneDrive同期問題の対策（0.5日）

#### 3.1 ビルド後に強制同期待ち

```matlab
% build_mex.m に追加
if ispc
    % OneDrive同期を待つ
    fprintf('Waiting for OneDrive sync...\n');
    pause(2);  % 2秒待機
end
```

#### 3.2 バイナリのハッシュ検証

```matlab
function hash = compute_mex_hash(mex_path)
    % ファイルハッシュを計算
    fid = fopen(mex_path, 'rb');
    data = fread(fid, inf, 'uint8');
    fclose(fid);
    
    % MD5ハッシュ
    md = java.security.MessageDigest.getInstance('MD5');
    hash = sprintf('%02x', typecast(md.digest(data), 'uint8'));
end

% ビルド後に検証
hash = compute_mex_hash(fullfile(bin_dir, 'mex_run_eskf.mexw64'));
fprintf('mex_run_eskf hash: %s\n', hash);
% このハッシュを両PCで比較
```

### Phase 4: 数値一貫性テスト（1日）

#### 4.1 決定論的テストケース

```matlab
function test_numeric_determinism()
    % 固定シードで再現可能なテスト
    rng(42, 'twister');
    
    % 事前生成されたテストデータを使用
    load('test_data_deterministic.mat', 'obs', 'expected_results');
    
    % 実行
    handle = mex_run_eskf('init', obs, 5.0, 0.0025);
    
    for k = 2001:2100  % 100ステップ
        mex_run_eskf('step', handle, obs, k);
    end
    
    state = mex_run_eskf('get_state', handle);
    
    % 期待値との比較
    p_error = max(abs(state.p - expected_results.p));
    bg_error = max(abs(state.bg - expected_results.bg));
    
    assert(p_error < 1e-6, 'Position mismatch: %e', p_error);
    assert(bg_error < 1e-6, 'Gyro bias mismatch: %e', bg_error);
    
    fprintf('✅ Numeric determinism test PASSED\n');
    
    mex_run_eskf('free', handle);
end
```

---

## 📊 成功基準

| Phase | 成功基準 | 検証方法 |
|-------|---------|---------|
| Phase 0 | 環境差を特定 | diagnose_mex_binary の出力比較 |
| Phase 1 | バイナリサイズ差 < 10% | ファイルサイズ比較 |
| Phase 2 | 全PCで同一結果 | ハッシュ比較 + 数値比較 |
| Phase 3 | 同期後に動作確認 | 再ビルドなしで実行成功 |
| Phase 4 | RMSE < 1e-10 (PC間差) | test_numeric_determinism |

---

## 🚨 即時実行アクション

1. **両PCで `diagnose_mex_binary` を実行**
   - コンパイラ情報を収集
   - 5ステップ後のジャイロバイアスを確認

2. **ノートPCでMEXを再ビルド**
   ```matlab
   cd kalman/cpp/build
   clear mex
   build_mex()
   clear mex
   ```

3. **バイナリサイズを確認**
   - 155KB前後になるべき
   - 980KBのままなら、コンパイラ設定を確認

4. **結果を比較**
   ```matlab
   cd kalman
   run_simulation(42, true)
   % Results/estimation_01.csv を比較
   ```

---

## 📝 次のステップ

1. `diagnose_mex_binary` の結果を収集
2. コンパイラが異なる場合は Phase 1.1 を実行
3. コンパイラが同じなのに結果が異なる場合は Phase 2 を実行
4. すべて一致したら Phase 4 で最終検証
