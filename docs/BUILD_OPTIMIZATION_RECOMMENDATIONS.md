# ビルド最適化と改善推奨事項

**作成日**: 2026-01-11  
**目的**: MinGW/MSVCビルドの最適化と保守性向上

---

## 🎯 現状の評価

### ✅ 正常動作している項目

1. **数値精度**: MinGW/MSVCともに同等（位置RMSE 0.32m）
2. **推定安定性**: batch_10sets で 100%成功
3. **コンパイラフラグ**: 浮動小数点演算の厳密性を確保
4. **型統一**: GPS以外は `float32` で統一

### ⚠️ 改善の余地がある項目

1. **バイナリサイズ**: MinGW (1.03 MB) vs MSVC (155 KB)
2. **ビルド時間**: 最適化レベルが高く、ビルドに時間がかかる
3. **デバッグ難易度**: デバッグシンボルが削除されている（`-s` フラグ）
4. **環境依存性**: コンパイラごとに異なるフラグ管理

---

## 📋 推奨改善事項（優先度順）

### Priority 1: デバッグビルドオプションの追加

#### 問題
現在は `-s` (strip) フラグで常にリリースビルドとなる。  
デバッグ時にシンボル情報がないため、クラッシュ解析が困難。

#### 解決策
```matlab
function build_mex(targets, debug_mode)
% BUILD_MEX  Build MEX files with optional debug mode
%
% Usage:
%  build_mex()           - リリースビルド（デフォルト）
%  build_mex([], true)   - デバッグビルド（シンボル保持）

if nargin < 2 || isempty(debug_mode)
    debug_mode = false;
end

% ... コンパイラ検出 ...

if is_msvc
    if debug_mode
        setenv('COMPFLAGS', '/Od /Zi /fp:precise /arch:SSE2 /MDd');
        opt_flags = {};
    else
        setenv('COMPFLAGS', '/O2 /fp:precise /arch:SSE2 /MD');
        opt_flags = {};
    end
else
    % MinGW
    if debug_mode
        opt_flags = {
            'CXXFLAGS=$CXXFLAGS -O0 -g -msse2 -mfpmath=sse -fno-fast-math -ffloat-store'
        };
    else
        opt_flags = {
            'CXXFLAGS=$CXXFLAGS -O2 -msse2 -mfpmath=sse -fno-fast-math -ffloat-store -s'
        };
    end
end

% ... 残りの処理 ...
end
```

**効果**:
- デバッグ時: シンボル保持、最適化無効 → クラッシュ解析が容易
- リリース時: 最適化有効、バイナリ縮小 → 実行速度向上

---

### Priority 2: MinGWバイナリサイズの削減（オプション）

#### 問題
MinGW静的リンクで 1.03 MB → MSVC動的リンク 155 KB の6.6倍

#### 解決策（慎重に検討）

##### Option A: 動的リンク化
```matlab
% MinGW用フラグに追加
opt_flags = {
    'CXXFLAGS=$CXXFLAGS -O2 -msse2 -mfpmath=sse -fno-fast-math -ffloat-store -s'
    'LINKFLAGS=$LINKFLAGS -shared-libgcc -shared-libstdc++'
};
```

**期待効果**:
- バイナリサイズ: 1.03 MB → ~200 KB (予想)

**リスク**:
- `libgcc_s_seh-1.dll`, `libstdc++-6.dll` への依存追加
- 配布時にDLLも必要（ユーザー環境に存在しない場合）

**推奨**: ❌ **非推奨** — 配布の複雑化に見合うメリットが少ない

##### Option B: リンク時最適化 (LTO)
```matlab
opt_flags = {
    'CXXFLAGS=$CXXFLAGS -O2 -msse2 -mfpmath=sse -fno-fast-math -ffloat-store -flto -s'
    'LINKFLAGS=$LINKFLAGS -flto'
};
```

**期待効果**:
- バイナリサイズ: 1.03 MB → ~800 KB (予想)
- 実行速度: 5-10% 向上

**リスク**:
- ビルド時間が2-3倍に増加
- 数値精度への影響（要検証）

**推奨**: ⚠️ **慎重に検討** — リリースビルドのみで試す

---

### Priority 3: ビルド設定の一元化

#### 問題
MSVCは `setenv('COMPFLAGS', ...)`, MinGWは `CXXFLAGS=$CXXFLAGS ...` と異なる記法。

#### 解決策
```matlab
% ビルド設定を構造体で管理
function flags = get_compiler_flags(compiler_type, debug_mode)
    flags = struct();
    
    if strcmp(compiler_type, 'msvc')
        if debug_mode
            flags.compile = '/Od /Zi /fp:precise /arch:SSE2 /MDd';
            flags.mex_opts = {};
        else
            flags.compile = '/O2 /fp:precise /arch:SSE2 /MD';
            flags.mex_opts = {};
        end
        flags.apply_method = 'setenv';
        
    elseif strcmp(compiler_type, 'mingw')
        base_flags = '-msse2 -mfpmath=sse -fno-fast-math -ffloat-store -frounding-math';
        if debug_mode
            cxx_flags = sprintf('-O0 -g %s', base_flags);
        else
            cxx_flags = sprintf('-O2 %s -s', base_flags);
        end
        flags.compile = cxx_flags;
        flags.mex_opts = {sprintf('CXXFLAGS=$CXXFLAGS %s', cxx_flags)};
        flags.apply_method = 'mex_opts';
    end
end

% build_mex.m 内で使用
flags = get_compiler_flags(is_msvc ? 'msvc' : 'mingw', debug_mode);
if strcmp(flags.apply_method, 'setenv')
    setenv('COMPFLAGS', flags.compile);
    compile_opts = [flags.mex_opts, {'-DNDEBUG', '-DKALMAN_NO_STANDALONE'}];
else
    compile_opts = [flags.mex_opts, {'-DNDEBUG', '-DKALMAN_NO_STANDALONE'}];
end
```

**効果**:
- 保守性向上（フラグ変更が1箇所で済む）
- コンパイラ追加時の対応が容易（将来のClang対応など）

---

### Priority 4: ビルド検証の自動化

#### 問題
ビルド後に `clear mex` を忘れると古いバイナリが使われる。

#### 解決策
```matlab
function build_mex(targets, debug_mode)
    % ... ビルド処理 ...
    
    % ビルド成功後の自動検証
    fprintf('\n=== ビルド検証 ===\n');
    for i = 1:numel(mex_targets)
        mex_name = mex_targets{i}{3};
        mex_path = fullfile(bin_dir, [mex_name '.mexw64']);
        
        % サイズチェック
        info = dir(mex_path);
        fprintf('%s: %d KB\n', mex_name, round(info.bytes/1024));
        
        % サニティチェック（MEX関数が呼べるか）
        clear mex;
        try
            % 簡易テスト（引数なしで呼んで、エラーメッセージが正しいか確認）
            feval(mex_name);
        catch ME
            if contains(ME.message, 'Not enough input arguments') || ...
               contains(ME.message, 'Invalid command') || ...
               contains(ME.message, 'Command must be')
                fprintf('  ✅ ロード成功\n');
            else
                fprintf('  ❌ ロード失敗: %s\n', ME.message);
            end
        end
    end
    
    fprintf('\nビルド完了。clear mex を実行しました。\n');
end
```

**効果**:
- ビルド直後にMEXキャッシュをクリア
- バイナリが正常にロードできるか確認

---

### Priority 5: クロスプラットフォーム対応の準備

#### 問題
現在はWindows専用（`.mexw64`のみ）。

#### 解決策（長期計画）

##### Phase 1: プラットフォーム検出
```matlab
function platform = detect_platform()
    if ispc
        platform = 'windows';
    elseif ismac
        platform = 'macos';
    elseif isunix
        platform = 'linux';
    else
        error('Unsupported platform');
    end
end

function ext = get_mex_extension(platform)
    switch platform
        case 'windows'
            ext = '.mexw64';
        case 'macos'
            if ismac && strcmp(computer, 'MACI64')
                ext = '.mexmaci64';  % Intel Mac
            else
                ext = '.mexmaca64';  % Apple Silicon
            end
        case 'linux'
            ext = '.mexa64';
        otherwise
            error('Unsupported platform');
    end
end
```

##### Phase 2: CMakeベースビルドシステム（将来）
```cmake
# CMakeLists.txt (例)
cmake_minimum_required(VERSION 3.15)
project(KalmanMEX)

find_package(Matlab REQUIRED)

# 共通コンパイラフラグ
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -msse2 -fno-fast-math")

# MEXターゲット定義
matlab_add_mex(
    NAME mex_run_eskf
    SRC mex_run_eskf.cpp eskf_core.cpp ...
    LINK_TO ${MATLAB_LIBRARIES}
)
```

**効果**:
- Linux/macOSでもビルド可能
- CIパイプライン（GitHub Actions）で自動テスト

---

## 🧪 推奨テスト手順

### 1. デバッグビルドのテスト
```matlab
% デバッグビルド
build_mex([], true);

% 簡易テスト
run_simulation(1, false);

% 問題があればデバッガで詳細解析
```

### 2. リリースビルドの回帰テスト
```matlab
% リリースビルド
build_mex([], false);

% 回帰テスト
run_batch_10sets();

% 期待結果: 10/10 成功、位置RMSE < 0.5m
```

### 3. 両コンパイラでの一致確認
```matlab
% MSVCでビルド
select_mex_compiler('msvc');
build_mex([], false);
run_batch_10sets();
movefile('Results/batch_10sets_summary.csv', 'Results/batch_msvc.csv');

% MinGWでビルド
select_mex_compiler('mingw');
build_mex([], false);
run_batch_10sets();
movefile('Results/batch_10sets_summary.csv', 'Results/batch_mingw.csv');

% 差分比較
msvc = readtable('Results/batch_msvc.csv');
mingw = readtable('Results/batch_mingw.csv');

diff_pos = max(abs(msvc.PosX_RMSE_m - mingw.PosX_RMSE_m));
fprintf('位置RMSE差分: %.6f m (期待: < 0.01m)\n', diff_pos);
```

---

## 📊 期待効果サマリー

| 改善項目 | 優先度 | 工数 | 期待効果 |
|---------|-------|------|---------|
| **デバッグビルド追加** | ⭐⭐⭐⭐⭐ | 0.5日 | デバッグ効率 3倍向上 |
| **ビルド設定一元化** | ⭐⭐⭐⭐ | 0.5日 | 保守性向上、バグ減少 |
| **ビルド検証自動化** | ⭐⭐⭐⭐ | 0.3日 | ビルドエラー早期発見 |
| **MinGWバイナリ縮小 (LTO)** | ⭐⭐ | 0.5日 | バイナリ 20%縮小（要検証） |
| **クロスプラットフォーム** | ⭐ | 3-5日 | Linux/macOS対応 |

---

## ⚠️ 注意事項

### 変更してはいけない項目

1. **浮動小数点フラグ**
   - `-fno-fast-math`, `/fp:precise` は**絶対に維持**
   - これを削除すると数値精度が不安定になる

2. **SSE2命令セット**
   - `-msse2`, `/arch:SSE2` は必須
   - 古いCPUでは動かないが、2003年以降のCPUなら問題なし

3. **NDEBUG定義**
   - `assert()` を無効化して実行速度を向上
   - デバッグビルドでは削除してOK

### 慎重に検討すべき項目

1. **最適化レベルの変更**
   - `-O3` や `/Ox` は数値誤差が増加する可能性
   - 変更後は必ず `run_batch_10sets()` で検証

2. **動的リンク化**
   - DLL依存を増やすと配布が複雑になる
   - 大学・企業環境ではDLL不足でエラーになるリスク

---

## 🏁 結論

**現在の実装は数値的に正常動作している。**

改善の余地はあるが、**機能的には問題なし**。  
以下の優先順位で段階的に改善を推奨：

1. **デバッグビルドオプション追加**（即座に実施推奨）
2. **ビルド設定の一元化**（保守性向上）
3. **ビルド検証の自動化**（品質保証）
4. **バイナリサイズ削減**（オプション、低優先度）
5. **クロスプラットフォーム対応**（長期計画）

---

**作成者**: GitHub Copilot  
**最終更新**: 2026-01-11
