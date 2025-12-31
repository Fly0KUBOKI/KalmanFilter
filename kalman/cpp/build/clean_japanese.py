#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import sys

def clean_japanese_comments(filepath):
    """Remove Japanese comments from C++ header file"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # List of replacements
        replacements = [
            # mex_run_eskf_sensor_updates.hpp
            ('前方宣言', 'Forward declaration'),
            ('前回のセンサー値', 'Previous sensor values'),
            ('// input_update_gps, input_noise_gps用', '// for input_update_gps, input_noise_gps'),
            ('// State変換', '// State conversion'),
            ('// デバッグ出力変換', '// Debug output conversion'),
            ('// フィールド数が12（innov, dxを含む）の場合のみ処理', '// Process only if field count >= 10 (includes innov, dx)'),
            ('センサー更新処理（mexCallMATLAB実装）', 'Sensor update processing (mexCallMATLAB implementation)'),
            ('GPS更新処理（mexCallMATLAB実装）', 'GPS update processing (mexCallMATLAB implementation)'),
            ('センサー更新の内部処理（mex_eskf_do_updateのhandle_update関数を統合）', 'Internal sensor update processing (integrated handle_update from mex_eskf_do_update)'),
            ('入力: sensor_type, meas, p, v, q, ba, bg, P, g, dt, [sample]', 'Input: sensor_type, meas, p, v, q, ba, bg, P, g, dt, [sample]'),
            ('出力: out_p, out_v, out_q, out_ba, out_bg, out_P, should_skip', 'Output: out_p, out_v, out_q, out_ba, out_bg, out_P, should_skip'),
            ('出力バッファを初期化', 'Initialize output buffers'),
            ('R取得 (C++ direct implementation)', 'Get R (C++ direct implementation)'),
            ('3x3行列から対角要素を取得', 'Get diagonal elements from 3x3 matrix'),
            ('ベクトル形式', 'Vector format'),
            ('スカラー（baroなど）', 'Scalar (e.g., baro)'),
            ('sensor_data構造体を構築', 'Build sensor_data struct'),
            ('mex_params構造体', 'mex_params struct'),
            ('センサータイプ別設定', 'Sensor type-specific settings'),
            ('state構造体', 'state struct'),
            ('MEUKFCore::step() 直接呼び出し（mex_meukf_step_v2を統合）', 'MEUKFCore::step() direct call (integrated mex_meukf_step_v2)'),
            ('MATLAB構造体からC++構造体への変換', 'Conversion from MATLAB struct to C++ struct'),
            ('MEUKFCore::step()を直接呼び出し', 'Direct call to MEUKFCore::step()'),
            ('C++構造体からMATLAB構造体への変換', 'Conversion from C++ struct to MATLAB struct'),
            ('互換性のため、innovフィールドを設定（既存コードが期待している）', 'Set innov field for compatibility (existing code expects it)'),
            ('dxフィールドを計算（dx = K * y、カルマンゲイン × イノベーション）', 'Calculate dx field (dx = K * y, Kalman gain * innovation)'),
            ('イノベーションがない場合は0', 'If no innovation, use 0'),
            ('noise estimate更新 (C++ direct implementation)', 'noise estimate update (C++ direct implementation)'),
            ('innov: output.last_yから直接取得', 'innov: get directly from output.last_y'),
        ]
        
        # Apply replacements
        for japanese, english in replacements:
            content = content.replace(japanese, english)
        
        # Write back
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)
        
        print(f'Cleaned: {filepath}')
        return True
        
    except Exception as e:
        print(f'Error processing {filepath}: {e}', file=sys.stderr)
        return False

def main():
    base_dir = r'c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\cpp\Inc\MEX'
    files_to_clean = [
        'mex_run_eskf_sensor_updates.hpp',
    ]
    
    for fname in files_to_clean:
        fpath = os.path.join(base_dir, fname)
        if os.path.exists(fpath):
            clean_japanese_comments(fpath)
        else:
            print(f'File not found: {fpath}')

if __name__ == '__main__':
    main()
