// Copied from Inc/ESKF/eskf_filter.hpp
#pragma once

#include "Common/filter_interface.hpp"
#include "../../Matrix/fixed_matrix.hpp"
#include "../../Quaternion/quaternion_functions.hpp"

namespace kalman {
namespace eskf {

using Matrix15x15 = cmath_fx::Matrix<15, 15, Scalar>;
using Vector15 = cmath_fx::Vector<15, Scalar>;

// ESKF状態構造体
struct ESKFState {
    Vector3 position;           // 位置
    Vector3 velocity;           // 速度
    Vector4 quaternion;         // クォータニオン
    Vector3 accel_bias;         // 加速度バイアス
    Vector3 gyro_bias;          // ジャイロバイアス
    Matrix15x15 P;              // 共分散行列
    
    // デフォルトコンストラクタ
    ESKFState()
        : position{0, 0, 0}
        , velocity{0, 0, 0}
        , quaternion{1, 0, 0, 0}
        , accel_bias{0, 0, 0}
        , gyro_bias{0, 0, 0}
    {
        P = Matrix15x15::identity();
    }
};

// ESKFフィルタクラス
class ESKFFilter {
public:
    ESKFFilter();
    ~ESKFFilter() = default;
    
    // 初期化
    void initialize(const ESKFState& initial_state, const FilterParams& params);
    
    // 統一update関数
    void update(const SensorInput& input, FilterOutput& output);
    
    // 状態リセット
    void reset();
    
    // 現在の状態を取得
    const ESKFState& get_state() const { return state_; }
    
private:
    ESKFState state_;
    FilterParams params_;
    SensorDataManager sensor_manager_;
    
    // 内部処理関数
    void predict(const SensorInput& input);
    void update_accel(const Vector3& accel);
    void update_mag(const Vector3& mag);
    void update_gps(const Vector3& gps_pos);
    void update_baro(Scalar pressure);
    
    // 状態→出力変換
    void state_to_output(FilterOutput& output) const;
};

} // namespace eskf
} // namespace kalman
