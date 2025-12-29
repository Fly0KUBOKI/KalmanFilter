# ビルド修正内容

## 問題

MEXファイルのソースコード移行後、以下のリンカーエラーが発生：

1. `mex_sensor_preprocessor.cpp`: `preprocess_accel`, `preprocess_mag`, `preprocess_baro`, `preprocess_gps`の未解決の外部シンボル
2. `mex_filter_management.cpp`: `setIdentityScaled`, `check_divergence`, `apply_zupt`の未解決の外部シンボル

## 原因

新しく作成した実装ファイル（`Src/Common/Sensor/sensor_preprocessor.cpp`、`Src/Common/filter_management.cpp`）がビルドに含まれていない。

## 修正内容

### 1. build_mex.mの修正

#### mex_sensor_preprocessor.cpp
```matlab
% 修正前
if wants('mex_sensor_preprocessor') && build_single_mex('mex_sensor_preprocessor.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)

% 修正後
sensor_preprocessor_cpp = fullfile(src_dir, 'Common', 'Sensor', 'sensor_preprocessor.cpp');
if wants('mex_sensor_preprocessor') && build_single_mex('mex_sensor_preprocessor.cpp', compile_opts, inc_args, {sensor_preprocessor_cpp}, bin_dir, [], log_fid)
```

#### mex_filter_management.cpp
```matlab
% 修正前
if exist('mex_filter_management.cpp', 'file')
    if wants('mex_filter_management') && build_single_mex('mex_filter_management.cpp', compile_opts, inc_args, {}, bin_dir, [], log_fid)

% 修正後
filter_management_cpp = fullfile(src_dir, 'Common', 'filter_management.cpp');
if exist('mex_filter_management.cpp', 'file')
    if wants('mex_filter_management') && build_single_mex('mex_filter_management.cpp', compile_opts, inc_args, {filter_management_cpp}, bin_dir, [], log_fid)
```

### 2. build_single_mex関数の改善

`extra_sources`のファイルが存在しない場合に警告を出すように修正：

```matlab
% 修正前
if exist(src, 'file')
    valid_extra_sources{end+1} = src;
end

% 修正後
if exist(src, 'file')
    valid_extra_sources{end+1} = src;
else
    warning('Extra source not found: %s', src);
end
```

また、`mex_file`を絶対パスに変換する処理を追加：

```matlab
% mex_fileを絶対パスに変換（mex_src_dirからの相対パスとして扱う）
build_dir = fileparts(mfilename('fullpath'));
cpp_root = fileparts(build_dir);
mex_src_dir = fullfile(cpp_root, 'MEX');
mex_file_full = fullfile(mex_src_dir, mex_file);
```

## 確認事項

1. ✅ `Src/Common/Sensor/sensor_preprocessor.cpp`が存在する
2. ✅ `Src/Common/filter_management.cpp`が存在する
3. ✅ `build_mex.m`でこれらのファイルが`extra_sources`に追加されている
4. ✅ `build_single_mex`関数が`extra_sources`を正しく処理している

## 次のステップ

1. `build_mex.m`を実行してビルドを確認
2. リンカーエラーが解消されることを確認
3. 動作テストを実施

