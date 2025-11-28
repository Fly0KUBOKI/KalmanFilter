````markdown
# ArduPilot Filter Library - 技術リファレンス

## Filter ライブラリ構造

### ディレクトリ構成
```
libraries/Filter/
├── LowPassFilter.h           # EMA ローパスフィルタ
├── LowPassFilter.cpp
├── LowPassFilter2p.h         # 2次 Biquad フィルタ
├── LowPassFilter2p.cpp
├── NotchFilter.h             # ノッチフィルタ
├── NotchFilter.cpp
├── AverageFilter.h           # 平均フィルタ
├── ModeFilter.h              # 中央値フィルタ
├── ModeFilter.cpp
├── DerivativeFilter.h        # 微分フィルタ
├── DerivativeFilter.cpp
├── FilterWithBuffer.h        # バッファ付きフィルタベース
├── FilterWithBuffer.cpp
├── FilterClass.h             # インターフェース
├── Butter.h                  # Butterworth係数
├── AP_Filter.h               # パラメータ管理
├── AP_Filter.cpp
├── AP_Filter_params.cpp
└── examples/
    └── LowPassFilter/
    └── LowPassFilter2p/
```

---

## 各フィルタの詳細仕様

### LowPassFilter（EMA ローパスフィルタ）

#### ヘッダファイル
```cpp
template <class T>
class DigitalLPF {
private:
    T output;
    bool initialised;
protected:
    T _apply(const T &sample, const float &alpha);
};

// 可変dt版
template <class T>
class LowPassFilter : public DigitalLPF<T> {
public:
    LowPassFilter(const float &cutoff_freq);
    void set_cutoff_frequency(const float &cutoff_freq);
    float get_cutoff_freq() const;
    T apply(const T &sample, const float &dt);
private:
    float cutoff_freq;
};

// 固定dt版
template <class T>
class LowPassFilterConstDt : public DigitalLPF<T> {
public:
    LowPassFilterConstDt(const float &sample_freq, const float &cutoff_freq);
    void set_cutoff_frequency(const float &sample_freq, const float &cutoff_freq);
    float get_cutoff_freq() const;
    T apply(const T &sample);
private:
    float cutoff_freq;
    float alpha;
};
```

````}