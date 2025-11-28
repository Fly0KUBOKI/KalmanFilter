````markdown
# ArduPilot センサーフィルタリング実装分析

*このドキュメントはArduPilotのセンサーフィルタリング、ノイズ除去、平滑化の実装を詳細に分析したものです*

## 目次

1. [フィルタの種類](#フィルタの種類)
2. [フィルタの構成](#フィルタの構成)
3. [センサー別の実装](#センサー別の実装)
4. [特殊な処理](#特殊な処理)
5. [実装の特徴](#実装の特徴)
6. [コードスニペット](#コードスニペット)

---

## フィルタの種類

### 1. Low Pass Filter (LPF) - ローパスフィルタ

#### 1.1 単純EMA (指数移動平均) フィルタ
**用途**: 可変時間ステップでの高速フィルタリング

```cpp
// LowPassFilter: 各サンプルで dt を提供
output = filter.apply(sample, dt);

// 実装原理: α = dt / (dt + RC), RC = 1/(2π*f_cutoff)
T _apply(const T &sample, const float &alpha) {
    output += (sample - output) * alpha;
    return output;
}
```

````