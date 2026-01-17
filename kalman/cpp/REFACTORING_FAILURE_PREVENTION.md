# リファクタリング失敗の再発防止ガイド

**作成日**: 2026年1月17日  
**関連インシデント**: GPS座標変換関数の変更による推定失敗（全10セットでRMSE異常）

---

## インシデント概要

### 症状
- 10セットすべてのシミュレーションで推定失敗
- 位置RMSEが異常に高い（最大 3347 km の誤差）
- 姿勢RMSEも一部で異常値（最大 148 deg）

### 直接的な原因
`sensor_preprocessor.cpp` を削除してヘッダーオンリー（`sensor_preprocessor.hpp`）化した際、GPS座標変換関数 `gps()` の実装を元の実装と異なる方法に置き換えてしまった。

#### 元の実装（正常動作）
```cpp
// Lib/Sensor/sensor_preprocessor.cpp (コミット 739a353)
PreprocessResult preprocess_gps(
    double lat, double lon, double alt,
    const cmath_fx::Vector<3, float>& origin,
    double buffer_tolerance
) {
    // ...
    double y_m = dlat / 9.0e-6;  // North
    double lat0rad = lat0 * common::math::PI / 180.0;
    double x_m = dlon / (9.0e-6 / std::cos(lat0rad));  // East
    double z_m = -dalt;  // Up (負の高度)
    
    result.output(0, 0) = y_m;   // 出力順序: [North, East, Up]
    result.output(1, 0) = x_m;
    result.output(2, 0) = z_m;
    return result;
}
```

#### 新しい実装（失敗）
```cpp
// Lib/Sensor/sensor_preprocessor.hpp (リファクタリング後)
inline Result gps(...) {
    // ...
    // coordinate_transform.hpp の lla_to_enu_simple() を使用
    cmath_fx::Vector<3, float> enu;
    coord::lla_to_enu_simple(lat, lon, alt, lat0, lon0, alt0, enu);
    result.output = enu;  // 出力順序: [East, North, Up] (WGS84楕円体変換)
    return result;
}
```

#### 重大な違い
| 項目 | 元の実装 | 新しい実装（失敗） |
|------|----------|------------------|
| 出力順序 | `[North, East, Up]` | `[East, North, Up]` |
| 変換方法 | 簡易線形変換 (`dlat/9.0e-6`) | WGS84楕円体変換 (`N0 * dlat`) |
| 高度の符号 | `-dalt` | `+dalt` |

この不一致により、**GPS位置推定が完全に破綻**した。

---

## 根本原因分析

### Why 1: なぜGPS変換関数を変更したのか？
- **直接原因**: `coordinate_transform.hpp` に新しく実装した `lla_to_enu_simple()` 関数を使いたかった
- **意図**: コードの統一・モジュール化（座標変換をcoordinate_transformモジュールに集約）
- **問題**: 元の実装との互換性を検証せずに置き換えた

### Why 2: なぜ互換性を検証しなかったのか？
- **直接原因**: リファクタリング時にテストを実行しなかった
- **問題**: ビルドが成功したら正常動作すると思い込んだ
- **背景**: C++コンパイラは座標順序や変換方法の違いを検出できない

### Why 3: なぜ元の実装と新しい実装で違いが生じたのか？
- **直接原因**: `coordinate_transform.hpp` の `lla_to_enu_simple()` を実装する際、標準的なENU座標系（East, North, Up）を採用した
- **問題**: 元の実装が**独自の座標順序**（`[North, East, Up]`）を使用していることに気づかなかった
- **背景**: ドキュメントやコメントに座標順序の明示的な記載がなかった

### Why 4: なぜドキュメントに座標順序が記載されていなかったのか？
- **直接原因**: 元の実装時にドキュメント化されていなかった
- **問題**: 暗黙の前提（「出力順序は関数内で決まる」）が共有されていなかった

### Why 5: なぜビルド後にテストを実行しなかったのか？
- **直接原因**: リファクタリング作業が多く、最後にまとめてテストする予定だった
- **問題**: 変更の影響範囲が大きい場合、段階的なテストが必要だった

---

## 再発防止策

### 即時実施（Critical）

#### 1. **座標系・変換関数の明示的ドキュメント化**
```cpp
/**
 * GPS LLA座標をローカルENU座標に変換
 * 
 * @param lat,lon,alt GPS座標（緯度[rad], 経度[rad], 高度[m]）
 * @param lat0,lon0,alt0 原点GPS座標
 * @return enu ローカルENU座標 [m]
 * 
 * **重要**: 出力順序は [North, East, Up] である（標準ENUとは異なる）
 * **重要**: 簡易線形変換を使用（WGS84楕円体は使わない）
 * **重要**: 高度は負の値（-dalt）で返却
 * 
 * 理由: 既存のフィルタ実装との互換性のため
 */
inline Result gps(...) {
    // ...
}
```

#### 2. **リファクタリング時のテストプロトコル**
```markdown
[ ] ビルド成功
[ ] 単体テスト実行（run_simulation(42, true)）
[ ] 結果CSV確認（位置RMSE < 1.0m, 姿勢RMSE < 1.0deg）
[ ] 回帰テスト（run_batch_10sets()）
[ ] 10セット中8セット以上PASS
```

- **ルール**: ビルド成功だけでは不十分。**必ず単体テストを実行**
- **ルール**: センサー処理・座標変換など、数値計算に関わる変更は**必ず回帰テスト**

#### 3. **単体テスト専用スクリプト作成**
```matlab
% quick_validation.m
% リファクタリング後の素早い動作確認用

clear mex;
fprintf('=== Quick Validation Test ===\n');
run_simulation(42, false);

% 結果確認
result = readtable('Results/estimation_01.csv');
pos_rmse = mean(sqrt(result.px.^2 + result.py.^2 + result.pz.^2));
fprintf('Position RMSE: %.4f m\n', pos_rmse);

if pos_rmse < 1.0
    fprintf('[PASS] 推定精度OK\n');
else
    fprintf('[FAIL] 推定精度異常！詳細確認が必要\n');
end
```

#### 4. **git pre-commit hook設定**
```bash
#!/bin/bash
# .git/hooks/pre-commit

# C++ファイルが変更されている場合、ビルド&テストを実行
if git diff --cached --name-only | grep -q "\.cpp$\|\.hpp$"; then
    echo "C++ files modified. Running build + quick test..."
    cd kalman/cpp/build
    matlab -batch "build_mex()" || exit 1
    cd ../../
    matlab -batch "run_simulation(42, false)" || exit 1
    echo "Test passed."
fi
```

### 短期実施（High Priority）

#### 5. **座標変換関数の単体テスト**
```cpp
// tests/test_coordinate_transform.cpp
TEST(CoordinateTransform, GPS_to_ENU_compatibility) {
    // 既知の入力・出力ペアでテスト
    double lat = 35.6895 * M_PI / 180.0;  // 東京
    double lon = 139.6917 * M_PI / 180.0;
    double alt = 10.0;
    double lat0 = lat, lon0 = lon, alt0 = 0.0;
    
    // 元の実装の期待値
    double expected_north = 0.0;  // dlat=0
    double expected_east = 0.0;   // dlon=0
    double expected_up = -10.0;   // -dalt
    
    // 新しい実装の出力
    sensor::preprocess::Result result = sensor::preprocess::gps(lat, lon, alt, origin, 1e-9);
    
    EXPECT_NEAR(result.output(0, 0), expected_north, 1e-3);
    EXPECT_NEAR(result.output(1, 0), expected_east, 1e-3);
    EXPECT_NEAR(result.output(2, 0), expected_up, 1e-3);
}
```

#### 6. **coordinate_transform.hpp のコメント修正**
```cpp
/**
 * 警告: この関数は標準的なENU座標系 [East, North, Up] を返します。
 * sensor_preprocessor.hpp の gps() 関数は独自の順序 [North, East, Up] を使用しています。
 * この2つの関数を混同しないでください！
 */
inline void lla_to_enu_simple(...) {
    enu(0, 0) = East;   // ← 明示的にコメント
    enu(1, 0) = North;
    enu(2, 0) = Up;
}
```

### 中長期実施（Medium Priority）

#### 7. **CI/CDパイプライン導入**
```yaml
# .github/workflows/cpp_build_test.yml
name: C++ Build & Test

on:
  push:
    paths:
      - 'kalman/cpp/**'
  pull_request:
    paths:
      - 'kalman/cpp/**'

jobs:
  build-and-test:
    runs-on: windows-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup MATLAB
        uses: matlab-actions/setup-matlab@v1
      - name: Build MEX
        run: |
          cd kalman/cpp/build
          matlab -batch "build_mex()"
      - name: Run Quick Test
        run: |
          cd kalman
          matlab -batch "run_simulation(42, false)"
      - name: Check RMSE
        run: |
          matlab -batch "quick_validation()"
```

#### 8. **型安全な座標構造体導入**
```cpp
// coordinate_types.hpp
namespace coord {

struct ENUCoord {  // East-North-Up (標準)
    float east, north, up;
};

struct NEUCoord {  // North-East-Up (独自)
    float north, east, up;
};

// 明示的な変換関数
inline NEUCoord enu_to_neu(const ENUCoord& enu) {
    return NEUCoord{enu.north, enu.east, enu.up};
}

} // namespace coord
```

---

## チェックリスト

### リファクタリング前
- [ ] 変更対象の関数の**入出力仕様**を文書化
- [ ] 座標系・単位・符号などの**暗黙の前提**を明示化
- [ ] 既存のテストケースがあれば実行して**基準値**を記録

### リファクタリング中
- [ ] 新しい実装と元の実装の**互換性**を確認
- [ ] 座標順序・変換係数・符号などの**細部**まで一致させる
- [ ] 不一致がある場合、**理由を明示的にドキュメント化**

### リファクタリング後
- [ ] **ビルド成功**を確認
- [ ] **単体テスト実行**（run_simulation(42, true)）
- [ ] 結果CSV確認：位置RMSE < 1.0m, 姿勢RMSE < 1.0deg
- [ ] **回帰テスト実行**（run_batch_10sets()）
- [ ] 10セット中**8セット以上PASS**を確認
- [ ] 失敗があれば、**即座に原因調査**

### コミット前
- [ ] テスト結果を**ログに記録**（Results/log/に保存）
- [ ] 変更内容を**コミットメッセージに明記**
- [ ] 互換性に影響がある場合、**BREAKING CHANGE**をマーク

---

## 教訓

1. **ビルド成功 ≠ 正常動作**  
   - C++コンパイラは座標順序や変換方法の違いを検出できない
   - 必ず実行時テストで動作を確認する

2. **暗黙の前提は明示化する**  
   - 座標系の順序（ENU vs NEU）
   - 単位（m, rad, deg）
   - 符号（+dalt vs -dalt）
   - これらをコメントやドキュメントに記載

3. **互換性を保つか、破るかを明確にする**  
   - 互換性を保つ場合: 元の実装と完全に一致させる
   - 互換性を破る場合: 明示的にドキュメント化し、全体を修正

4. **段階的なテストを実施する**  
   - 小さな変更ごとにテスト
   - 大きなリファクタリングは複数のコミットに分割
   - 各コミットでテストをパス

5. **数値計算は特に慎重に**  
   - センサー処理・座標変換・フィルタアルゴリズムなど
   - 小さな違いが大きな誤差につながる
   - 必ず回帰テストを実行

---

## 関連ドキュメント

- [docs/CPP_INPUT_OUTPUT_SPEC.md](../docs/CPP_INPUT_OUTPUT_SPEC.md) - 型マッピング・配列レイアウト
- [kalman/cpp/Lib/Sensor/README.md](Lib/Sensor/README.md) - Sensorモジュール仕様
- [docs/CODING_STANDARDS.md](../docs/CODING_STANDARDS.md) - コーディング規約

---

**最終更新**: 2026年1月17日  
**レビュアー**: 今後のリファクタリング時にこのガイドを参照すること
