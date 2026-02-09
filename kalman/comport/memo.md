

  

Created 11 todos

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport) の読み取り

まず、CSVファイルのフォーマットとPython読み取りスクリプトを確認します。   

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport/read_sensor_data.py#1-1) の 1 から 100 行を読み取り

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport/config.py) の読み取り

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport/README.md#1-1) の 1 から 100 行を読み取り

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport/stillness.csv#1-1) の 1 から 10 行を読み取り

CSVフォーマットを確認しました。ジャイロ値が静止時に非常に大きい(~30 deg/s)ことに気づきました。全CSVを包括的に分析するスクリプトを作成します。 

Starting: *各CSVデータの読み込み・概観* (2/11)

Configuring a Python Environment
Completed with input: {
  "resourcePath": "c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter\\kalman\\comport"
}

仮想環境を作成しています
Completed with input: {
  "resourcePath": "c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter\\kalman\\comport"
}

パッケージ matplotlib, numpy, pandas, scipy を Python 3.13.7 (.venv) にインストールしています
Completed with input: {
  "packageList": [
    "pandas",
    "numpy",
    "scipy",
    "matplotlib"
  ],
  "resourcePath": "c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter"
}

まず全CSVの概要統計を取得する包括的な分析スクリプトを作成します。 

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport/analyze_sensor_data.py) が作成されました

Ran terminal command:  cd "c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\comport" && "C:/Users/takut/OneDrive/ドキュメント/MATLAB/KalmanFilter/.venv/Scripts/python.exe" analyze_sensor_data.py

[](file:///c%3A/Users/takut/AppData/Roaming/Code/User/workspaceStorage/cf9bbd98d7229d3737590c49d674a56a/GitHub.copilot-chat/chat-session-resources/238649ff-b106-4958-874f-9cf9ead8d584/toolu_vrtx_01VVAs1iAJm93qZ962wT9V2q__vscode-1770593440710/content.txt#1-1) の 1 から 600 行を読み取り

[](file:///c%3A/Users/takut/AppData/Roaming/Code/User/workspaceStorage/cf9bbd98d7229d3737590c49d674a56a/GitHub.copilot-chat/chat-session-resources/238649ff-b106-4958-874f-9cf9ead8d584/toolu_vrtx_01VVAs1iAJm93qZ962wT9V2q__vscode-1770593440710/content.txt#600-600) の 600 から 650 行を読み取り

初期分析結果が揃いました。非常に重要な発見がいくつかあります。次に、シミュレーションパラメータと詳細比較を行います。 

Starting: *詳細分析スクリプト作成* (3/5)

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/GenerateData/config_params.m#1-1) の 1 から 100 行を読み取り

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/GenerateData/config_params.m#100-100) の 100 から 151 行を読み取り

次にシミュレーションのセンサーモデル生成コードも確認します。  

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/GenerateData/generate_sensor_observations.m#1-1) の 1 から 200 行を読み取り

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/GenerateData/add_sensor_noise.m#1-1) の 1 から 200 行を読み取り

シミュレーションコードの確認が完了しました。非常に重要な発見があります。次に詳細な検証スクリプトを作成します。 

Starting: *深層検証スクリプト作成* (4/5)

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport/deep_analysis.py) が作成されました

Ran terminal command:  cd "c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\comport" && "C:/Users/takut/OneDrive/ドキュメント/MATLAB/KalmanFilter/.venv/Scripts/python.exe" deep_analysis.py

[](file:///c%3A/Users/takut/AppData/Roaming/Code/User/workspaceStorage/cf9bbd98d7229d3737590c49d674a56a/GitHub.copilot-chat/chat-session-resources/238649ff-b106-4958-874f-9cf9ead8d584/toolu_vrtx_01WQ5FpSSKpBgVSJavoVA7Mr__vscode-1770593440720/content.txt) の読み取り

Starting: *総合分析・レポート作成* (5/5)

Tilt rightの初回解析ではRoll≈87°の区間を検出していたのに深層解析では検出されていないので、もう少しデータを確認します。 

Magnetic field deep analysis

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/GenerateData/generate_sensor_observations.m#120-120) の 120 から 160 行を読み取り

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport/config.py) の読み取り

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport/Clockwise.csv#1-1) の 1 から 10 行を読み取り

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport/stillness.csv#1-1) の 1 から 10 行を読み取り

テキスト `mag_strength` (`**/kalman/GenerateData/**`) を検索し、結果は 2 件でした

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/GenerateData/config_params.m#1-1) の 1 から 100 行を読み取り

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/comport/read_sensor_data.py#1-1) の 1 から 60 行を読み取り

[](file:///c%3A/Users/takut/OneDrive/%E3%83%89%E3%82%AD%E3%83%A5%E3%83%A1%E3%83%B3%E3%83%88/MATLAB/KalmanFilter/kalman/GenerateData/generate_sensor_observations.m#1-1) の 1 から 50 行を読み取り

Configuring a Python Environment
Completed with input: {
  "resourcePath": "c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter"
}

`pylanceRunCodeSnippet` を実行しました
Completed with input: {
  "workspaceRoot": "c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter",
  "codeSnippet": "\nimport numpy as np\nimport csv\n\n# ===== Helper: read CSV =====\ndef read_csv(path):\n    with open(path, 'r') as f:\n        reader = csv.DictReader(f)\n        data = {col: [] for col in reader.fieldnames}\n        for row in reader:\n            for col in reader.fieldnames:\n                data[col].append(float(row[col]))\n    return {k: np.array(v) for k, v in data.items()}\n\n# ===== 1. Load Clockwise.csv =====\ncw = read_csv(r'c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter\\kalman\\comport\\Clockwise.csv')\nprint(f\"Clockwise.csv: {len(cw['time'])} samples, time range: {cw['time'][0]:.3f} - {cw['time'][-1]:.3f} s\")\nprint(f\"  mag_x range: [{cw['mag_x'].min():.2f}, {cw['mag_x'].max():.2f}] μT\")\nprint(f\"  mag_y range: [{cw['mag_y'].min():.2f}, {cw['mag_y'].max():.2f}] μT\")\nprint(f\"  mag_z range: [{cw['mag_z'].min():.2f}, {cw['mag_z'].max():.2f}] μT\")\n\n# ===== 2. Load stillness.csv =====\nst = read_csv(r'c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter\\kalman\\comport\\stillness.csv')\nprint(f\"\\nstillness.csv: {len(st['time'])} samples, time range: {st['time'][0]:.3f} - {st['time'][-1]:.3f} s\")\nprint(f\"  mag_x: mean={st['mag_x'].mean():.3f}, std={st['mag_x'].std():.3f} μT\")\nprint(f\"  mag_y: mean={st['mag_y'].mean():.3f}, std={st['mag_y'].std():.3f} μT\")\nprint(f\"  mag_z: mean={st['mag_z'].mean():.3f}, std={st['mag_z'].std():.3f} μT\")\n\nprint(\"\\n\" + \"=\"*80)\nprint(\"ANALYSIS REPORT: Magnetic Field Model Validation\")\nprint(\"=\"*80)\n\n# =====================================================================\n# SECTION A: Circle Fit (Hard-Iron & Soft-Iron) on Clockwise rotation\n# =====================================================================\nprint(\"\\n\" + \"-\"*60)\nprint(\"A. MAGNETIC CIRCLE FIT (Clockwise.csv - mag_x vs mag_y)\")\nprint(\"-\"*60)\n\nmx = cw['mag_x']\nmy = cw['mag_y']\n\n# Algebraic circle fit: (x - cx)^2 + (y - cy)^2 = r^2\n# => x^2 + y^2 = 2*cx*x + 2*cy*y + (r^2 - cx^2 - cy^2)\n# => [x, y, 1] * [2*cx; 2*cy; r^2 - cx^2 - cy^2] = x^2 + y^2\nA = np.column_stack([mx, my, np.ones_like(mx)])\nb = mx**2 + my**2\nresult = np.linalg.lstsq(A, b, rcond=None)\nparams_fit = result[0]\ncx = params_fit[0] / 2.0\ncy = params_fit[1] / 2.0\nr_sq = params_fit[2] + cx**2 + cy**2\nr = np.sqrt(max(r_sq, 0))\n\ndistances = np.sqrt((mx - cx)**2 + (my - cy)**2)\nresiduals = distances - r\nrmse = np.sqrt(np.mean(residuals**2))\nmax_res = np.max(np.abs(residuals))\n\nprint(f\"  Circle center (hard-iron offset): cx={cx:.3f} μT, cy={cy:.3f} μT\")\nprint(f\"  Circle radius: {r:.3f} μT\")\nprint(f\"  Fit RMSE: {rmse:.4f} μT\")\nprint(f\"  Max residual: {max_res:.4f} μT\")\nprint(f\"  Distance range: [{distances.min():.3f}, {distances.max():.3f}] μT\")\n\n# Eccentricity / Soft-iron analysis via ellipse fit\n# Fit ellipse: Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0\n# with constraint A + C = 1 (to prevent trivial solution)\nD_ell = np.column_stack([mx**2, mx*my, my**2, mx, my, np.ones_like(mx)])\n# SVD approach\nU, S, Vt = np.linalg.svd(D_ell)\nell_params = Vt[-1, :]  # last row of Vt = smallest singular value\nA_e, B_e, C_e, D_e, E_e, F_e = ell_params\n\n# Semi-axes from general conic\n# The discriminant = B^2 - 4AC\ndisc = B_e**2 - 4*A_e*C_e\nprint(f\"\\n  Ellipse fit parameters: A={A_e:.6f}, B={B_e:.6f}, C={C_e:.6f}\")\nprint(f\"  Discriminant (B²-4AC): {disc:.6f} (negative = ellipse)\")\n\n# Calculate semi-axes\nif disc < 0:\n    # It's an ellipse\n    # Center\n    x0_e = (B_e*E_e - 2*C_e*D_e) / (4*A_e*C_e - B_e**2)\n    y0_e = (B_e*D_e - 2*A_e*E_e) / (4*A_e*C_e - B_e**2)\n    \n    # Semi-axes\n    M = np.array([[A_e, B_e/2], [B_e/2, C_e]])\n    eigvals = np.linalg.eigvalsh(M)\n    \n    # Numerator for semi-axes\n    num = -(A_e*x0_e**2 + B_e*x0_e*y0_e + C_e*y0_e**2 + D_e*x0_e + E_e*y0_e + F_e)\n    a_semi = np.sqrt(abs(num / eigvals[0]))\n    b_semi = np.sqrt(abs(num / eigvals[1]))\n    \n    if a_semi < b_semi:\n        a_semi, b_semi = b_semi, a_semi\n    \n    eccentricity = np.sqrt(1 - (b_semi/a_semi)**2) if a_semi > 0 else 0\n    axis_ratio = b_semi / a_semi if a_semi > 0 else 1\n    \n    # Rotation angle\n    theta = 0.5 * np.arctan2(B_e, A_e - C_e)\n    \n    print(f\"  Ellipse center: ({x0_e:.3f}, {y0_e:.3f}) μT\")\n    print(f\"  Semi-major axis: {a_semi:.3f} μT\")\n    print(f\"  Semi-minor axis: {b_semi:.3f} μT\")\n    print(f\"  Axis ratio (b/a): {axis_ratio:.4f}\")\n    print(f\"  Eccentricity: {eccentricity:.4f}\")\n    print(f\"  Rotation angle: {np.degrees(theta):.2f}°\")\n    \n    if axis_ratio > 0.95:\n        print(f\"  → Soft-iron distortion: NEGLIGIBLE (axis ratio ≈ 1)\")\n    elif axis_ratio > 0.85:\n        print(f\"  → Soft-iron distortion: MODERATE\")\n    else:\n        print(f\"  → Soft-iron distortion: SIGNIFICANT\")\n\n# Angular coverage\nangles = np.degrees(np.arctan2(my - cy, mx - cx))\nangle_range = angles.max() - angles.min()\nprint(f\"\\n  Angular coverage: {angle_range:.1f}° (full rotation = 360°)\")\n\n# Check how much of the circle was covered\nangle_bins = np.zeros(36)  # 10-degree bins\nfor a in angles:\n    bin_idx = int((a % 360) / 10) % 36\n    angle_bins[bin_idx] += 1\ncovered_bins = np.sum(angle_bins > 0)\nprint(f\"  Angular bins covered: {covered_bins}/36 ({covered_bins/36*100:.0f}%)\")\n\n# =====================================================================\n# SECTION B: Dip Angle from stillness.csv\n# =====================================================================\nprint(\"\\n\" + \"-\"*60)\nprint(\"B. MAGNETIC DIP ANGLE (stillness.csv)\")\nprint(\"-\"*60)\n\nmx_s = st['mag_x']\nmy_s = st['mag_y']\nmz_s = st['mag_z']\n\n# Compute horizontal magnitude\nm_horiz = np.sqrt(mx_s**2 + my_s**2)\nm_total = np.sqrt(mx_s**2 + my_s**2 + mz_s**2)\n\n# Dip angle = arctan2(mz, sqrt(mx^2 + my^2))\n# But note: in body frame, if device is level, then body frame ≈ NED frame\n# Check if device is level using accelerometer\nax_s = st['accel_x']\nay_s = st['accel_y']\naz_s = st['accel_z']\n\nroll_acc = np.degrees(np.arctan2(ay_s.mean(), az_s.mean()))\npitch_acc = np.degrees(np.arctan2(-ax_s.mean(), np.sqrt(ay_s.mean()**2 + az_s.mean()**2)))\n\nprint(f\"  Device orientation from accel:\")\nprint(f\"    Roll  = {roll_acc:.2f}°\")\nprint(f\"    Pitch = {pitch_acc:.2f}°\")\n\n# Magnetic field in body frame\nmx_mean = mx_s.mean()\nmy_mean = my_s.mean()\nmz_mean = mz_s.mean()\nm_total_mean = np.sqrt(mx_mean**2 + my_mean**2 + mz_mean**2)\nm_horiz_mean = np.sqrt(mx_mean**2 + my_mean**2)\n\nprint(f\"\\n  Mean magnetic field (body frame):\")\nprint(f\"    mag_x = {mx_mean:.3f} μT\")\nprint(f\"    mag_y = {my_mean:.3f} μT\")\nprint(f\"    mag_z = {mz_mean:.3f} μT\")\nprint(f\"    |M_horiz| = {m_horiz_mean:.3f} μT\")\nprint(f\"    |M_total| = {m_total_mean:.3f} μT\")\n\n# dip angle (positive = downward into earth in Northern hemisphere)\n# If sensor Z axis points UP, then magnetic Z component is negative in Northern hemisphere\n# But our data shows mz positive (~7-8 μT), which means either:\n# 1. The sensor Z axis points DOWN, or\n# 2. The hard-iron offset shifts the Z value\ndip_angle_raw = np.degrees(np.arctan2(mz_mean, m_horiz_mean))\nprint(f\"\\n  Raw dip angle (arctan2(mz, horiz)): {dip_angle_raw:.2f}°\")\n\n# Hard-iron corrected dip angle - use circle fit center from Clockwise\n# For Z, we need a separate estimate. Use the average during full rotation.\nmz_cw_mean = cw['mag_z'].mean()\nprint(f\"  Mean mag_z during Clockwise rotation: {mz_cw_mean:.3f} μT (hard-iron Z estimate)\")\n\n# Corrected values (subtract hard-iron)\nmx_corr = mx_mean - cx\nmy_corr = my_mean - cy\nmz_corr = mz_mean - mz_cw_mean  # approximate Z hard-iron from rotation average\nm_horiz_corr = np.sqrt(mx_corr**2 + my_corr**2)\n\nprint(f\"\\n  Hard-iron corrected magnetic field:\")\nprint(f\"    mag_x_corr = {mx_corr:.3f} μT\")\nprint(f\"    mag_y_corr = {my_corr:.3f} μT\")\nprint(f\"    mag_z_corr = {mz_corr:.3f} μT\")\nprint(f\"    |M_horiz_corr| = {m_horiz_corr:.3f} μT\")\n\ndip_angle_corr = np.degrees(np.arctan2(mz_corr, m_horiz_corr))\nprint(f\"  Corrected dip angle: {dip_angle_corr:.2f}°\")\nprint(f\"\\n  Japan expected magnetic dip angle: ~49° (Tokyo area)\")\nprint(f\"  Japan expected total field: ~46-50 μT\")\n\n# The total field magnitude\nm_total_corr = np.sqrt(mx_corr**2 + my_corr**2 + mz_corr**2)\nprint(f\"  Corrected total field: {m_total_corr:.3f} μT\")\n\n# =====================================================================\n# SECTION C: Unit Consistency Analysis\n# =====================================================================\nprint(\"\\n\" + \"-\"*60)\nprint(\"C. UNIT CONSISTENCY ANALYSIS\")\nprint(\"-\"*60)\nprint(f\"  Simulation model: mag_strength = 50, comment says 'nT'\")\nprint(f\"  Simulation model: mag_world = [50, 0, 0] (North-only, no dip)\")\nprint(f\"  config.py: Mag columns labeled [μT]\")\nprint(f\"  config.py: EXPECTED_RANGES Mag: (-100, 100) μT\")\nprint(f\"\")\nprint(f\"  Real sensor data:\")\nprint(f\"    Stillness total field: {m_total_mean:.2f} μT\")\nprint(f\"    This is consistent with Japan's ~46-50 μT\")\nprint(f\"\")\nprint(f\"  Simulation comparison:\")\nprint(f\"    If mag_strength=50 is in nT: 50 nT = 0.05 μT → WAY TOO SMALL\")\nprint(f\"    If mag_strength=50 is in μT: 50 μT → matches Japan's field ✓\")\nprint(f\"\")\nprint(f\"  CONCLUSION: The comment '% nT' is WRONG. The value 50 represents μT.\")\nprint(f\"  50 μT = 50,000 nT. Japan's magnetic field is approximately:\")\nprint(f\"  - Total intensity: ~46,500 nT (46.5 μT) in Tokyo\")\nprint(f\"  - Horizontal component: ~30,000 nT (30 μT)\")\nprint(f\"  - Vertical component: ~35,500 nT (35.5 μT)\")\nprint(f\"  → mag_strength=50 μT is a reasonable approximation.\")\n\n# =====================================================================\n# SECTION D: Simulation Model Problems\n# =====================================================================\nprint(\"\\n\" + \"-\"*60)\nprint(\"D. SIMULATION MODEL DEFICIENCIES\")\nprint(\"-\"*60)\nprint(f\"  1. NO DIP ANGLE: mag_world = [50, 0, 0]\")\nprint(f\"     → All magnetic field is horizontal (North component only)\")\nprint(f\"     → mag_z in body frame is always 0 when device is level\")\nprint(f\"     → Real data shows significant Z component (~{mz_mean:.1f} μT at stillness)\")\nprint(f\"\")\nprint(f\"  2. NO HARD-IRON MODEL: Simulation has no bias offset\")\nprint(f\"     → Real sensor has hard-iron: cx={cx:.1f}, cy={cy:.1f} μT\")\nprint(f\"     → This means the magnetometer reads non-zero even without external field changes\")\nprint(f\"\")\nprint(f\"  3. UNIT COMMENT ERROR: Code says 'nT' but value is μT\")\nprint(f\"\")\n\n# Correct model for Japan\nI_deg = 49.0  # inclination/dip for Japan (Tokyo)\nD_deg = -7.5  # declination for Japan (Tokyo)\nF_total = 46.5  # total field in μT\n\n# NED components with declination and inclination\nm_north = F_total * np.cos(np.radians(I_deg)) * np.cos(np.radians(D_deg))\nm_east = F_total * np.cos(np.radians(I_deg)) * np.sin(np.radians(D_deg))\nm_down = F_total * np.sin(np.radians(I_deg))\n\nprint(f\"  RECOMMENDED mag_world for Japan (Tokyo):\")\nprint(f\"    Inclination (dip): {I_deg}°\")\nprint(f\"    Declination: {D_deg}°\")\nprint(f\"    Total field: {F_total} μT\")\nprint(f\"    mag_world = [{m_north:.2f}, {m_east:.2f}, {m_down:.2f}] μT (NED)\")\nprint(f\"    → North: {m_north:.2f} μT\")\nprint(f\"    → East: {m_east:.2f} μT\")\nprint(f\"    → Down: {m_down:.2f} μT\")\n\nprint()\n"
}

[](file:///c%3A/Users/takut/AppData/Roaming/Code/User/workspaceStorage/cf9bbd98d7229d3737590c49d674a56a/GitHub.copilot-chat/chat-session-resources/238649ff-b106-4958-874f-9cf9ead8d584/toolu_vrtx_01K6pSfdCRcw4eahxMNKkBkz__vscode-1770593440733/content.txt#1-1) の 1 から 200 行を読み取り

[](file:///c%3A/Users/takut/AppData/Roaming/Code/User/workspaceStorage/cf9bbd98d7229d3737590c49d674a56a/GitHub.copilot-chat/chat-session-resources/238649ff-b106-4958-874f-9cf9ead8d584/toolu_vrtx_01K6pSfdCRcw4eahxMNKkBkz__vscode-1770593440733/content.txt#200-200) の 200 から 368 行を読み取り

`pylanceRunCodeSnippet` を実行しました
Completed with input: {
  "codeSnippet": "\nimport numpy as np\nimport csv\nimport sys\nimport io\n\n# Force UTF-8 output\nsys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')\n\ndef read_csv(path):\n    with open(path, 'r') as f:\n        reader = csv.DictReader(f)\n        data = {col: [] for col in reader.fieldnames}\n        for row in reader:\n            for col in reader.fieldnames:\n                data[col].append(float(row[col]))\n    return {k: np.array(v) for k, v in data.items()}\n\ncw = read_csv(r'c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter\\kalman\\comport\\Clockwise.csv')\nst = read_csv(r'c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter\\kalman\\comport\\stillness.csv')\n\nprint(f\"Clockwise.csv: {len(cw['time'])} samples, time {cw['time'][0]:.3f}-{cw['time'][-1]:.3f} s\")\nprint(f\"  mag_x: [{cw['mag_x'].min():.2f}, {cw['mag_x'].max():.2f}] uT\")\nprint(f\"  mag_y: [{cw['mag_y'].min():.2f}, {cw['mag_y'].max():.2f}] uT\")\nprint(f\"  mag_z: [{cw['mag_z'].min():.2f}, {cw['mag_z'].max():.2f}] uT\")\nprint(f\"stillness.csv: {len(st['time'])} samples, time {st['time'][0]:.3f}-{st['time'][-1]:.3f} s\")\n\n# ============= A. CIRCLE FIT =================\nprint(\"\\n=== A. MAGNETIC CIRCLE FIT (Clockwise mag_x vs mag_y) ===\")\nmx = cw['mag_x']\nmy = cw['mag_y']\n\nA = np.column_stack([mx, my, np.ones_like(mx)])\nb = mx**2 + my**2\nparams_fit = np.linalg.lstsq(A, b, rcond=None)[0]\ncx = params_fit[0] / 2.0\ncy = params_fit[1] / 2.0\nr_sq = params_fit[2] + cx**2 + cy**2\nr = np.sqrt(max(r_sq, 0))\n\ndistances = np.sqrt((mx - cx)**2 + (my - cy)**2)\nresiduals = distances - r\nrmse = np.sqrt(np.mean(residuals**2))\n\nprint(f\"  Hard-iron offset: cx={cx:.3f}, cy={cy:.3f} uT\")\nprint(f\"  Circle radius: {r:.3f} uT\")\nprint(f\"  Fit RMSE: {rmse:.4f} uT (radius = {r:.1f}, RMSE/radius = {rmse/r*100:.1f}%)\")\nprint(f\"  Max residual: {np.max(np.abs(residuals)):.4f} uT\")\nprint(f\"  Distance range: [{distances.min():.3f}, {distances.max():.3f}] uT\")\n\n# Ellipse fit via SVD\nD_ell = np.column_stack([mx**2, mx*my, my**2, mx, my, np.ones_like(mx)])\nU, S, Vt = np.linalg.svd(D_ell)\nA_e, B_e, C_e, D_e, E_e, F_e = Vt[-1, :]\ndisc = B_e**2 - 4*A_e*C_e\nprint(f\"  Ellipse discriminant (B^2-4AC): {disc:.6f} (negative=ellipse)\")\n\nif disc < 0:\n    x0_e = (B_e*E_e - 2*C_e*D_e) / (4*A_e*C_e - B_e**2)\n    y0_e = (B_e*D_e - 2*A_e*E_e) / (4*A_e*C_e - B_e**2)\n    M = np.array([[A_e, B_e/2], [B_e/2, C_e]])\n    eigvals = np.linalg.eigvalsh(M)\n    num = -(A_e*x0_e**2 + B_e*x0_e*y0_e + C_e*y0_e**2 + D_e*x0_e + E_e*y0_e + F_e)\n    a_semi = np.sqrt(abs(num / eigvals[0]))\n    b_semi = np.sqrt(abs(num / eigvals[1]))\n    if a_semi < b_semi:\n        a_semi, b_semi = b_semi, a_semi\n    axis_ratio = b_semi / a_semi\n    ecc = np.sqrt(1 - axis_ratio**2)\n    theta = 0.5 * np.arctan2(B_e, A_e - C_e)\n    print(f\"  Ellipse center: ({x0_e:.3f}, {y0_e:.3f}) uT\")\n    print(f\"  Semi-major: {a_semi:.3f}, Semi-minor: {b_semi:.3f} uT\")\n    print(f\"  Axis ratio (b/a): {axis_ratio:.4f}\")\n    print(f\"  Eccentricity: {ecc:.4f}\")\n    print(f\"  Rotation angle: {np.degrees(theta):.2f} deg\")\n    if axis_ratio > 0.95:\n        print(f\"  -> Soft-iron: NEGLIGIBLE (near circular)\")\n    elif axis_ratio > 0.85:\n        print(f\"  -> Soft-iron: MODERATE\")\n    else:\n        print(f\"  -> Soft-iron: SIGNIFICANT\")\n\n# Angular coverage\nangles = np.arctan2(my - cy, mx - cx)\nangles_deg = np.degrees(angles)\nangle_bins = np.zeros(36)\nfor a in angles_deg:\n    angle_bins[int((a % 360) / 10) % 36] += 1\ncovered = np.sum(angle_bins > 0)\nprint(f\"  Angular bins covered: {covered}/36 ({covered/36*100:.0f}%)\")\n\n# ============= B. DIP ANGLE =================\nprint(\"\\n=== B. MAGNETIC DIP ANGLE (stillness.csv) ===\")\nmx_s, my_s, mz_s = st['mag_x'], st['mag_y'], st['mag_z']\nax_s, ay_s, az_s = st['accel_x'], st['accel_y'], st['accel_z']\n\nroll_acc = np.degrees(np.arctan2(ay_s.mean(), az_s.mean()))\npitch_acc = np.degrees(np.arctan2(-ax_s.mean(), np.sqrt(ay_s.mean()**2 + az_s.mean()**2)))\nprint(f\"  Device orientation: Roll={roll_acc:.2f} deg, Pitch={pitch_acc:.2f} deg\")\n\nmx_m, my_m, mz_m = mx_s.mean(), my_s.mean(), mz_s.mean()\nm_h = np.sqrt(mx_m**2 + my_m**2)\nm_t = np.sqrt(mx_m**2 + my_m**2 + mz_m**2)\nprint(f\"  Mean mag field (body): x={mx_m:.3f}, y={my_m:.3f}, z={mz_m:.3f} uT\")\nprint(f\"  |M_horiz|={m_h:.3f}, |M_total|={m_t:.3f} uT\")\n\ndip_raw = np.degrees(np.arctan2(mz_m, m_h))\nprint(f\"  Raw dip angle: {dip_raw:.2f} deg\")\n\n# Hard-iron corrected\nmz_cw_mean = cw['mag_z'].mean()\nprint(f\"  Clockwise mean mag_z (Z hard-iron est): {mz_cw_mean:.3f} uT\")\nmx_c, my_c, mz_c = mx_m - cx, my_m - cy, mz_m - mz_cw_mean\nm_h_c = np.sqrt(mx_c**2 + my_c**2)\nm_t_c = np.sqrt(mx_c**2 + my_c**2 + mz_c**2)\ndip_corr = np.degrees(np.arctan2(mz_c, m_h_c))\nprint(f\"  Corrected mag: x={mx_c:.3f}, y={my_c:.3f}, z={mz_c:.3f} uT\")\nprint(f\"  Corrected |M_horiz|={m_h_c:.3f}, |M_total|={m_t_c:.3f} uT\")\nprint(f\"  Corrected dip angle: {dip_corr:.2f} deg\")\nprint(f\"  Japan expected dip: ~49 deg (Tokyo)\")\nprint(f\"  Japan expected total field: ~46.5 uT\")\n\n# Also try: rotate body mag to NED using accel-derived roll/pitch\nroll_r = np.radians(roll_acc)\npitch_r = np.radians(pitch_acc)\n# Rotation body->NED (using roll and pitch only, no yaw correction)\ncr, sr = np.cos(roll_r), np.sin(roll_r)\ncp, sp = np.cos(pitch_r), np.sin(pitch_r)\n# Level the magnetometer (tilt compensation)\nmx_level = mx_m * cp + my_m * sr * sp + mz_m * cr * sp\nmy_level = my_m * cr - mz_m * sr\nmz_level = -mx_m * sp + my_m * sr * cp + mz_m * cr * cp\nm_h_level = np.sqrt(mx_level**2 + my_level**2)\ndip_level = np.degrees(np.arctan2(mz_level, m_h_level))\nprint(f\"\\n  Tilt-compensated mag (NED approx): N={mx_level:.3f}, E={my_level:.3f}, D={mz_level:.3f}\")\nprint(f\"  Tilt-compensated dip angle: {dip_level:.2f} deg\")\n\n# For a better dip estimate: use hard-iron corrected + tilt compensated\nmx_ct = mx_c * cp + my_c * sr * sp + mz_c * cr * sp\nmy_ct = my_c * cr - mz_c * sr\nmz_ct = -mx_c * sp + my_c * sr * cp + mz_c * cr * cp\nm_h_ct = np.sqrt(mx_ct**2 + my_ct**2)\ndip_ct = np.degrees(np.arctan2(mz_ct, m_h_ct))\nprint(f\"  HI-corrected + tilt-comp dip: {dip_ct:.2f} deg\")\n\n# ============= C. UNIT ANALYSIS =================\nprint(\"\\n=== C. UNIT CONSISTENCY ===\")\nprint(f\"  Simulation: mag_strength = 50 (comment: nT)\")\nprint(f\"  Simulation: mag_world = [50, 0, 0]\")\nprint(f\"  config.py:  Mag unit = [uT], range = (-100,100)\")\nprint(f\"  Real total field: {m_t:.2f} uT (matches Japan ~46.5 uT)\")\nprint(f\"  If 50 nT: 0.05 uT -> 1000x too small\")\nprint(f\"  If 50 uT: matches real data order of magnitude\")\nprint(f\"  CONCLUSION: Comment is WRONG. Value 50 = uT (not nT)\")\nprint(f\"  Also: noise 'mag_std=5.0' and outlier range 'mag=50' use same unit\")\n\n# ============= D. HEADING COMPARISON =================\nprint(\"\\n=== D. HEADING COMPARISON (Clockwise) ===\")\nt = cw['time']\ndt_arr = np.diff(t)\n# Median dt to check sampling rate\ndt_med = np.median(dt_arr)\nprint(f\"  Median dt: {dt_med:.4f} s ({1/dt_med:.1f} Hz)\")\n\n# Static period for gyro bias estimation: 2.7 to 5.4 s\nstatic_mask = (t >= 2.7) & (t <= 5.4)\nn_static = np.sum(static_mask)\ngz_static = cw['gyro_z'][static_mask]\ngz_bias = gz_static.mean()\ngx_bias = cw['gyro_x'][static_mask].mean()\ngy_bias = cw['gyro_y'][static_mask].mean()\nprint(f\"  Static period (2.7-5.4s): {n_static} samples\")\nprint(f\"  Gyro bias: gx={gx_bias:.3f}, gy={gy_bias:.3f}, gz={gz_bias:.3f} deg/s\")\n\n# Magnetometer heading: atan2(-mag_y, mag_x)\n# Need to subtract hard-iron first\nmx_hi = mx - cx\nmy_hi = my - cy\nheading_mag = np.degrees(np.arctan2(-my_hi, mx_hi))\n# Unwrap to avoid jumps\nheading_mag_unwrap = np.unwrap(np.radians(np.arctan2(-my_hi, mx_hi)))\nheading_mag_unwrap = np.degrees(heading_mag_unwrap)\n\n# Gyro integration: heading = integral(gyro_z - bias) * dt\ngz_corr = cw['gyro_z'] - gz_bias  # deg/s after bias removal\nheading_gyro = np.zeros(len(t))\nfor i in range(1, len(t)):\n    dt_i = t[i] - t[i-1]\n    heading_gyro[i] = heading_gyro[i-1] + gz_corr[i] * dt_i\n\n# Set initial heading from magnetometer\nheading_gyro += heading_mag_unwrap[0]\n\n# Total heading change\nmag_total = heading_mag_unwrap[-1] - heading_mag_unwrap[0]\ngyro_total = heading_gyro[-1] - heading_gyro[0]\nprint(f\"\\n  Magnetometer heading range: {heading_mag_unwrap.min():.1f} to {heading_mag_unwrap.max():.1f} deg\")\nprint(f\"  Total mag heading change: {mag_total:.1f} deg\")\nprint(f\"  Total gyro heading change: {gyro_total:.1f} deg\")\nprint(f\"  Difference (gyro - mag): {gyro_total - mag_total:.1f} deg\")\n\n# Time series comparison at key points\nn = len(t)\nfor frac_label, frac in [(\"Start\", 0), (\"25%\", 0.25), (\"50%\", 0.5), (\"75%\", 0.75), (\"End\", -1)]:\n    idx = int(frac * n) if frac >= 0 else n-1\n    print(f\"    {frac_label}: t={t[idx]:.2f}s, mag_hdg={heading_mag_unwrap[idx]:.1f}, gyro_hdg={heading_gyro[idx]:.1f}, diff={heading_gyro[idx]-heading_mag_unwrap[idx]:.1f} deg\")\n\n# Drift analysis\n# Compute running difference\nhdg_diff = heading_gyro - heading_mag_unwrap\nprint(f\"\\n  Heading difference (gyro - mag) statistics:\")\nprint(f\"    Mean: {hdg_diff.mean():.2f} deg\")\nprint(f\"    Std:  {hdg_diff.std():.2f} deg\")\nprint(f\"    Max:  {hdg_diff.max():.2f} deg\")\nprint(f\"    Min:  {hdg_diff.min():.2f} deg\")\n\n# Check for rotation segments\nprint(f\"\\n  Rotation segments (large |gyro_z - bias|):\")\nrotation_mask = np.abs(gz_corr) > 10  # more than 10 deg/s\nrotation_segments = []\nin_segment = False\nseg_start = 0\nfor i in range(len(t)):\n    if rotation_mask[i] and not in_segment:\n        in_segment = True\n        seg_start = i\n    elif not rotation_mask[i] and in_segment:\n        in_segment = False\n        rotation_segments.append((seg_start, i-1))\nif in_segment:\n    rotation_segments.append((seg_start, len(t)-1))\n\nfor j, (s, e) in enumerate(rotation_segments[:5]):\n    hdg_change_mag = heading_mag_unwrap[e] - heading_mag_unwrap[s]\n    hdg_change_gyro = heading_gyro[e] - heading_gyro[s]\n    print(f\"    Segment {j+1}: t={t[s]:.2f}-{t[e]:.2f}s, mag={hdg_change_mag:.1f}deg, gyro={hdg_change_gyro:.1f}deg, diff={hdg_change_gyro-hdg_change_mag:.1f}deg\")\n\n# ============= E. SIMULATION MODEL DEFICIENCIES =================\nprint(\"\\n=== E. SIMULATION MODEL ISSUES ===\")\nprint(f\"  1. NO DIP ANGLE: mag_world = [50, 0, 0] - pure North\")\nprint(f\"     Real sensor shows mag_z ~ {mz_m:.1f} uT at stillness\")\nprint(f\"     With hard-iron correction + tilt comp, dip ~ {dip_ct:.1f} deg\")\nprint(f\"     Japan expected: ~49 deg\")\nprint(f\"\")\nprint(f\"  2. NO HARD-IRON MODEL:\")\nprint(f\"     Real hard-iron: x={cx:.1f}, y={cy:.1f}, z~{mz_cw_mean:.1f} uT\")\nprint(f\"     Simulation assumes zero offset\")\nprint(f\"\")\nprint(f\"  3. UNIT COMMENT: 'nT' should be 'uT'\")\nprint(f\"\")\nI_deg, D_deg, F_t = 49.0, -7.5, 46.5\nmn = F_t * np.cos(np.radians(I_deg)) * np.cos(np.radians(D_deg))\nme = F_t * np.cos(np.radians(I_deg)) * np.sin(np.radians(D_deg))\nmd = F_t * np.sin(np.radians(I_deg))\nprint(f\"  RECOMMENDED mag_world for Japan (Tokyo):\")\nprint(f\"    mag_world = [{mn:.2f}, {me:.2f}, {md:.2f}] uT  (NED)\")\nprint(f\"    (I={I_deg} deg, D={D_deg} deg, F={F_t} uT)\")\n\n# ============= F. CIRCLE FIT QUALITY SUMMARY =================\nprint(\"\\n=== F. SUMMARY ===\")\nprint(f\"  Circle fit quality: RMSE/radius = {rmse/r*100:.1f}% -> {'GOOD' if rmse/r < 0.05 else 'MODERATE' if rmse/r < 0.10 else 'POOR'}\")\nprint(f\"  Hard-iron X: {cx:.2f} uT\")\nprint(f\"  Hard-iron Y: {cy:.2f} uT\")\nprint(f\"  Hard-iron Z (approx): {mz_cw_mean:.2f} uT\")\nprint(f\"  Soft-iron axis ratio: {axis_ratio:.4f} -> {'negligible' if axis_ratio > 0.95 else 'present'}\")\nprint(f\"  Dip angle (best est): {dip_ct:.1f} deg (expected ~49 deg)\")\nprint(f\"  Total field (corr): {m_t_c:.1f} uT (expected ~46.5 uT)\")\nprint(f\"  Mag heading total change: {mag_total:.1f} deg\")\nprint(f\"  Gyro heading total change: {gyro_total:.1f} deg\")\nprint(f\"  Gyro-Mag drift over session: {hdg_diff[-1]-hdg_diff[0]:.1f} deg\")\n",
  "workspaceRoot": "c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter"
}

[](file:///c%3A/Users/takut/AppData/Roaming/Code/User/workspaceStorage/cf9bbd98d7229d3737590c49d674a56a/GitHub.copilot-chat/chat-session-resources/238649ff-b106-4958-874f-9cf9ead8d584/toolu_vrtx_01FKgfTq7eY3WjBeSdE73kaS__vscode-1770593440736/content.txt#1-1) の 1 から 250 行を読み取り

[](file:///c%3A/Users/takut/AppData/Roaming/Code/User/workspaceStorage/cf9bbd98d7229d3737590c49d674a56a/GitHub.copilot-chat/chat-session-resources/238649ff-b106-4958-874f-9cf9ead8d584/toolu_vrtx_01FKgfTq7eY3WjBeSdE73kaS__vscode-1770593440736/content.txt#250-250) の 250 から 396 行を読み取り

`pylanceRunCodeSnippet` を実行しました
Completed with input: {
  "codeSnippet": "\nimport numpy as np\nimport csv\nimport sys\nimport io\n\nsys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')\n\ndef read_csv(path):\n    with open(path, 'r') as f:\n        reader = csv.DictReader(f)\n        data = {col: [] for col in reader.fieldnames}\n        for row in reader:\n            for col in reader.fieldnames:\n                data[col].append(float(row[col]))\n    return {k: np.array(v) for k, v in data.items()}\n\ncw = read_csv(r'c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter\\kalman\\comport\\Clockwise.csv')\nst = read_csv(r'c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter\\kalman\\comport\\stillness.csv')\n\nmx = cw['mag_x']\nmy = cw['mag_y']\nt = cw['time']\n\n# Circle fit\nA = np.column_stack([mx, my, np.ones_like(mx)])\nb = mx**2 + my**2\npf = np.linalg.lstsq(A, b, rcond=None)[0]\ncx = pf[0] / 2.0\ncy = pf[1] / 2.0\nr = np.sqrt(max(pf[2] + cx**2 + cy**2, 0))\n\n# Hard-iron corrected\nmx_hi = mx - cx\nmy_hi = my - cy\nmz_cw_mean = cw['mag_z'].mean()\n\n# ===== FIXED HEADING CALCULATION =====\n# arctan2 returns RADIANS. No need to call np.radians again!\nheading_mag_rad = np.arctan2(-my_hi, mx_hi)  # radians\nheading_mag_unwrap = np.degrees(np.unwrap(heading_mag_rad))  # unwrap in radians, then to degrees\n\n# Gyro bias from static period 2.7-5.4s\nstatic_mask = (t >= 2.7) & (t <= 5.4)\ngz_bias = cw['gyro_z'][static_mask].mean()\ngx_bias = cw['gyro_x'][static_mask].mean()\ngy_bias = cw['gyro_y'][static_mask].mean()\n\n# Gyro integrated heading\ngz_corr = cw['gyro_z'] - gz_bias\nheading_gyro = np.zeros(len(t))\nfor i in range(1, len(t)):\n    dt_i = t[i] - t[i-1]\n    heading_gyro[i] = heading_gyro[i-1] + gz_corr[i] * dt_i\nheading_gyro += heading_mag_unwrap[0]\n\nmag_total = heading_mag_unwrap[-1] - heading_mag_unwrap[0]\ngyro_total = heading_gyro[-1] - heading_gyro[0]\n\nprint(\"=== HEADING COMPARISON (FIXED) ===\")\nprint(f\"  Sampling: {1/np.median(np.diff(t)):.1f} Hz\")\nprint(f\"  Gyro bias (static 2.7-5.4s): gx={gx_bias:.2f}, gy={gy_bias:.2f}, gz={gz_bias:.2f} deg/s\")\nprint(f\"  Mag heading range: {heading_mag_unwrap.min():.1f} to {heading_mag_unwrap.max():.1f} deg\")\nprint(f\"  Total mag heading change: {mag_total:.1f} deg\")\nprint(f\"  Total gyro heading change: {gyro_total:.1f} deg\")\nprint(f\"  Diff (gyro - mag total): {gyro_total - mag_total:.1f} deg\")\nprint()\n\n# Time series at key points\nn = len(t)\nfor label, frac in [(\"Start\", 0), (\"25%\", 0.25), (\"50%\", 0.5), (\"75%\", 0.75), (\"End\", -1)]:\n    idx = int(frac * n) if frac >= 0 else n-1\n    d = heading_gyro[idx] - heading_mag_unwrap[idx]\n    print(f\"  {label:5s}: t={t[idx]:6.2f}s  mag={heading_mag_unwrap[idx]:8.1f}  gyro={heading_gyro[idx]:8.1f}  diff={d:7.1f} deg\")\n\n# Statistics\nhdg_diff = heading_gyro - heading_mag_unwrap\nprint(f\"\\n  Heading diff (gyro-mag):\")\nprint(f\"    Mean: {hdg_diff.mean():.2f} deg\")\nprint(f\"    Std:  {hdg_diff.std():.2f} deg\")\nprint(f\"    Max:  {np.max(np.abs(hdg_diff)):.2f} deg\")\n\n# Rotation segments analysis\nrotation_mask = np.abs(gz_corr) > 10\nsegments = []\nin_seg = False\nfor i in range(len(t)):\n    if rotation_mask[i] and not in_seg:\n        in_seg = True; start = i\n    elif not rotation_mask[i] and in_seg:\n        in_seg = False; segments.append((start, i-1))\nif in_seg:\n    segments.append((start, len(t)-1))\n\nprint(f\"\\n  Rotation segments (|gz_corr| > 10 deg/s): {len(segments)} segments\")\nfor j, (s, e) in enumerate(segments[:10]):\n    dm = heading_mag_unwrap[e] - heading_mag_unwrap[s]\n    dg = heading_gyro[e] - heading_gyro[s]\n    dur = t[e] - t[s]\n    print(f\"    Seg {j+1}: {t[s]:.2f}-{t[e]:.2f}s ({dur:.2f}s), mag={dm:.1f}, gyro={dg:.1f}, diff={dg-dm:.1f} deg\")\n\n# Are there multiple distinct rotation episodes?\nprint(f\"\\n  Rotation episodes (gaps > 1s between segments):\")\nepisodes = []\nep_start = segments[0][0] if segments else 0\nep_end = segments[0][1] if segments else 0\nfor j in range(1, len(segments)):\n    if t[segments[j][0]] - t[segments[j-1][1]] > 1.0:\n        episodes.append((ep_start, ep_end))\n        ep_start = segments[j][0]\n    ep_end = segments[j][1]\nif segments:\n    episodes.append((ep_start, ep_end))\n\nfor j, (s, e) in enumerate(episodes):\n    dm = heading_mag_unwrap[e] - heading_mag_unwrap[s]\n    dg = heading_gyro[e] - heading_gyro[s]\n    dur = t[e] - t[s]\n    print(f\"    Episode {j+1}: {t[s]:.2f}-{t[e]:.2f}s ({dur:.1f}s), mag={dm:.1f}, gyro={dg:.1f} deg\")\n\n# Is gyro heading monotonically decreasing (clockwise)?\ndecreasing_frac = np.mean(np.diff(heading_gyro) < 0)\nprint(f\"\\n  Gyro heading decreasing fraction: {decreasing_frac*100:.1f}% (expect ~high for clockwise)\")\n\n# Stillness check\nmx_s, my_s, mz_s = st['mag_x'], st['mag_y'], st['mag_z']\nax_s, ay_s, az_s = st['accel_x'], st['accel_y'], st['accel_z']\n\n# Better dip angle estimate from stillness:\n# The key issue is that the hard-iron values measured from Clockwise might include\n# the dip angle effect. Let's compute dip from the circle fit parameters differently.\n# \n# During clockwise rotation (pure yaw), if device stays level:\n# - mag_x_body and mag_y_body trace a circle (horizontal field component)\n# - mag_z_body stays constant (vertical field component)\n# The circle radius = horizontal field magnitude\n# The mean mag_z during rotation (with any hard-iron averaged out if mixed rot/still) = vertical + HI_z\n# \n# But we can't separate vertical field from Z hard-iron with yaw-only rotation!\n# We need either: pitch rotation or known Z field reference.\n\nhoriz_field = r  # circle radius = horizontal field after HI correction\nvert_field_candidates = [mz_cw_mean]  # this includes both real vertical + HI_z\n\n# If device is level during stillness, mag_z_body = field_down + HI_z\n# During yaw rotation, mag_z is constant (if level) = field_down + HI_z\n# So we can't separate them. Let's note this.\n\nprint(f\"\\n=== DIP ANGLE ANALYSIS ===\")\nprint(f\"  Horizontal field (from circle radius): {horiz_field:.2f} uT\")\nprint(f\"  mag_z during rotation (avg): {mz_cw_mean:.2f} uT\")\nprint(f\"  mag_z during stillness (avg): {mz_s.mean():.2f} uT\")\nprint(f\"  mag_z is NOT the vertical field directly (includes Z hard-iron)\")\nprint(f\"  CANNOT separate dip from Z hard-iron with yaw-only rotation data\")\nprint(f\"\")\nprint(f\"  BUT we can bound the dip angle:\")\nprint(f\"  If Z hard-iron = 0: dip = arctan2({mz_cw_mean:.2f}, {horiz_field:.2f}) = {np.degrees(np.arctan2(mz_cw_mean, horiz_field)):.1f} deg\")\nprint(f\"  Expected Japan dip: ~49 deg\")\nprint(f\"  Required vertical for 49 deg dip: {horiz_field * np.tan(np.radians(49)):.1f} uT\")\nprint(f\"  That implies Z hard-iron = {mz_cw_mean:.1f} - {horiz_field * np.tan(np.radians(49)):.1f} = {mz_cw_mean - horiz_field * np.tan(np.radians(49)):.1f} uT\")\nprint(f\"\")\n\n# Japan field reference\nprint(f\"  Japan (Tokyo) expected values:\")\nprint(f\"    Total: ~46.5 uT, Horiz: ~30 uT, Vert: ~35.5 uT\")\nprint(f\"  Measured horizontal (circle radius): {horiz_field:.1f} uT (expected ~30)\")\nprint(f\"  Ratio measured/expected horiz: {horiz_field/30:.2f}\")\n\n# Total field from sensor\nm_total_raw = np.sqrt(mx.mean()**2 + my.mean()**2 + cw['mag_z'].mean()**2)\nprint(f\"  Raw total field (Clockwise avg): {m_total_raw:.1f} uT\")\n# Better: compute per-sample total field\nm_total_per = np.sqrt(mx**2 + my**2 + cw['mag_z']**2)\nprint(f\"  Per-sample total field: mean={m_total_per.mean():.2f}, std={m_total_per.std():.2f} uT\")\n# After hard-iron correction\nm_total_corr = np.sqrt(mx_hi**2 + my_hi**2 + (cw['mag_z'] - mz_cw_mean)**2)\nprint(f\"  HI-corrected total field: mean={m_total_corr.mean():.2f}, std={m_total_corr.std():.2f} uT\")\nprint(f\"  HI-corrected total = circle_radius = {horiz_field:.1f} (since mz-mz_mean -> 0)\")\nprint(f\"  This is ONLY the horizontal field! Real total should be ~46.5 uT\")\n",
  "workspaceRoot": "c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter"
}

[](file:///c%3A/Users/takut/AppData/Roaming/Code/User/workspaceStorage/cf9bbd98d7229d3737590c49d674a56a/GitHub.copilot-chat/chat-session-resources/238649ff-b106-4958-874f-9cf9ead8d584/toolu_vrtx_01U75mWj1UZCAiFvU9qLzNiP__vscode-1770593440739/content.txt#155-155) の 155 から 266 行を読み取り

非常に詳細な磁気場解析結果が得られました。最後の仕上げとして、Upwardデータの重力ノルムの異常（|g|≈8.43）についてと、Tilt rightの正確な姿勢変化について追加確認します。  

`pylanceRunCodeSnippet` を実行しました
Completed with input: {
  "workspaceRoot": "c:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter",
  "codeSnippet": "\nimport pandas as pd\nimport numpy as np\n\n# Upward.csv の accel_z=0 問題\ndf = pd.read_csv(r'C:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter\\kalman\\comport\\Upward.csv')\nax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values\nt = df['time'].values\n\n# accel_z = 0 のサンプル数\nzero_az = np.sum(np.abs(az) < 0.001)\nprint(f\"accel_z ≈ 0 のサンプル数: {zero_az} / {len(az)}\")\nprint(f\"accel_z の最小3つ: {sorted(az)[:5]}\")\n\n# 0でないサンプルだけで再計算\nmask_nonzero = np.abs(az) > 0.01\nif np.sum(mask_nonzero) > 0:\n    ax_nz = ax[mask_nonzero]\n    ay_nz = ay[mask_nonzero]\n    az_nz = az[mask_nonzero]\n    g_mag_nz = np.sqrt(ax_nz**2 + ay_nz**2 + az_nz**2)\n    print(f\"\\naz≠0 のサンプル数: {np.sum(mask_nonzero)}\")\n    print(f\"|g| (az≠0): {np.mean(g_mag_nz):.4f}\")\n\n# 最初の静止区間(0.8-9.9s)は本当に水平か？\nmask_s = (t >= 0.8) & (t <= 9.0)\nax_s, ay_s, az_s = ax[mask_s], ay[mask_s], az[mask_s]\ng_s = np.array([np.mean(ax_s), np.mean(ay_s), np.mean(az_s)])\nprint(f\"\\n最初の静止区間 (0.8-9.0s):\")\nprint(f\"  accel平均: [{g_s[0]:.4f}, {g_s[1]:.4f}, {g_s[2]:.4f}]\")\nprint(f\"  |g|: {np.linalg.norm(g_s):.4f}\")\n\n# 中盤の傾斜区間 (14-32s)  \nmask_m = (t >= 14.0) & (t <= 32.0)\nax_m, ay_m, az_m = ax[mask_m], ay[mask_m], az[mask_m]\ng_m = np.array([np.mean(ax_m), np.mean(ay_m), np.mean(az_m)])\ngn_m = g_m / np.linalg.norm(g_m)\npitch_m = np.degrees(np.arctan2(-gn_m[0], np.sqrt(gn_m[1]**2 + gn_m[2]**2)))\nprint(f\"\\n中盤傾斜区間 (14-32s):\")\nprint(f\"  accel平均: [{g_m[0]:.4f}, {g_m[1]:.4f}, {g_m[2]:.4f}]\")\nprint(f\"  |g|: {np.linalg.norm(g_m):.4f}\")\nprint(f\"  Pitch: {pitch_m:.1f}°\")\n\n# accel_z = 0 の出現位置を確認\nzero_indices = np.where(np.abs(az) < 0.001)[0]\nif len(zero_indices) > 0:\n    print(f\"\\naccel_z≈0 の位置:\")\n    for idx in zero_indices[:10]:\n        print(f\"  t={t[idx]:.3f}s, accel=[{ax[idx]:.4f}, {ay[idx]:.4f}, {az[idx]:.6f}]\")\n\nprint(\"\\n--- Tilt right.csv 姿勢詳細 ---\")\ndf2 = pd.read_csv(r'C:\\Users\\takut\\OneDrive\\ドキュメント\\MATLAB\\KalmanFilter\\kalman\\comport\\Tilt right.csv')\nt2 = df2['time'].values\nax2, ay2, az2 = df2['accel_x'].values, df2['accel_y'].values, df2['accel_z'].values\n\n# 各時刻のRollとPitchを計算\ng_mag2 = np.sqrt(ax2**2 + ay2**2 + az2**2)\nroll2 = np.degrees(np.arctan2(ay2, az2))\npitch2 = np.degrees(np.arctan2(-ax2, np.sqrt(ay2**2 + az2**2)))\n\n# 主要な姿勢値\nprint(f\"Roll 範囲: [{np.min(roll2):.1f}°, {np.max(roll2):.1f}°]\")\nprint(f\"Pitch 範囲: [{np.min(pitch2):.1f}°, {np.max(pitch2):.1f}°]\")\n\n# Roll > 60° の区間\nhigh_roll_mask = np.abs(roll2) > 60\nif np.sum(high_roll_mask) > 0:\n    print(f\"Roll>60° のサンプル数: {np.sum(high_roll_mask)}\")\n    t_high = t2[high_roll_mask]\n    print(f\"Roll>60° の時間範囲: {t_high[0]:.1f}s - {t_high[-1]:.1f}s\")\n    \n    # Roll>60°での安定区間（2秒以上連続して大きい）\n    # 代表的な静止区間(21-24s)を確認\n    mask_r = (t2 >= 21.0) & (t2 <= 24.0)\n    g_r = np.array([np.mean(ax2[mask_r]), np.mean(ay2[mask_r]), np.mean(az2[mask_r])])\n    gn = g_r / np.linalg.norm(g_r)\n    print(f\"\\n右傾斜安定区間 (21-24s):\")\n    print(f\"  accel平均: [{g_r[0]:.4f}, {g_r[1]:.4f}, {g_r[2]:.4f}]\")\n    print(f\"  |g|: {np.linalg.norm(g_r):.4f}\")\n    print(f\"  Roll: {np.degrees(np.arctan2(gn[1], gn[2])):.1f}°\")\n    print(f\"  Pitch: {np.degrees(np.arctan2(-gn[0], np.sqrt(gn[1]**2 + gn[2]**2))):.1f}°\")\n    print(f\"  理論(Roll=90°): accel=[0, ±9.81, 0]\")\n    print(f\"  → Y軸が {np.mean(ay2[mask_r]):.2f} (≈9.73) → ✅ Right tilt確認\")\n"
}

Ran terminal command:  cd "c:\Users\takut\OneDrive\ドキュメント\MATLAB\KalmanFilter\kalman\comport" && "C:/Users/takut/OneDrive/ドキュメント/MATLAB/KalmanFilter/.venv/Scripts/python.exe" -c "
import pandas as pd, numpy as np, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

df = pd.read_csv('Upward.csv')
ax, ay, az = df['accel_x'].values, df['accel_y'].values, df['accel_z'].values
t = df['time'].values

zero_az = np.sum(np.abs(az) < 0.001)
print(f'accel_z near 0: {zero_az} / {len(az)}')
print(f'accel_z min 5: {sorted(az)[:5]}')

mask_nz = np.abs(az) > 0.01
g_nz = np.sqrt(ax[mask_nz]**2 + ay[mask_nz]**2 + az[mask_nz]**2)
print(f'|g| (az!=0): {np.mean(g_nz):.4f}, count={np.sum(mask_nz)}')

mask_s = (t >= 0.8) & (t <= 9.0)
g_s = np.array([np.mean(ax[mask_s]), np.mean(ay[mask_s]), np.mean(az[mask_s])])
print(f'Initial static: [{g_s[0]:.4f}, {g_s[1]:.4f}, {g_s[2]:.4f}], |g|={np.linalg.norm(g_s):.4f}')

mask_m = (t >= 19.0) & (t <= 32.0)  
g_m = np.array([np.mean(ax[mask_m]), np.mean(ay[mask_m]), np.mean(az[mask_m])])
gn = g_m / np.linalg.norm(g_m)
pitch = np.degrees(np.arctan2(-gn[0], np.sqrt(gn[1]**2 + gn[2]**2)))
print(f'Mid tilt (19-32s): [{g_m[0]:.4f}, {g_m[1]:.4f}, {g_m[2]:.4f}], |g|={np.linalg.norm(g_m):.4f}, Pitch={pitch:.1f}')

zeros = np.where(np.abs(az) < 0.001)[0]
for idx in zeros[:5]:
    print(f'  zero at t={t[idx]:.3f}, a=[{ax[idx]:.4f},{ay[idx]:.4f},{az[idx]:.6f}]')

print()
df2 = pd.read_csv('Tilt right.csv')
t2, ax2, ay2, az2 = df2['time'].values, df2['accel_x'].values, df2['accel_y'].values, df2['accel_z'].values
roll2 = np.degrees(np.arctan2(ay2, az2))
print(f'Tilt right Roll range: [{np.min(roll2):.1f}, {np.max(roll2):.1f}]')
hr = np.abs(roll2) > 60
print(f'Roll>60 count: {np.sum(hr)}')
if np.sum(hr) > 0:
    t_h = t2[hr]
    print(f'Roll>60 time: {t_h[0]:.1f}s - {t_h[-1]:.1f}s')

mask_r = (t2 >= 21.0) & (t2 <= 24.0)
g_r = np.array([np.mean(ax2[mask_r]), np.mean(ay2[mask_r]), np.mean(az2[mask_r])])
gn_r = g_r / np.linalg.norm(g_r)
roll_r = np.degrees(np.arctan2(gn_r[1], gn_r[2]))
print(f'Tilt right 21-24s: accel=[{g_r[0]:.3f},{g_r[1]:.3f},{g_r[2]:.3f}], |g|={np.linalg.norm(g_r):.4f}, Roll={roll_r:.1f}')
"

全ての検証データが揃いました。Tilt right で Roll=88.2° を確認でき、Upward で Pitch=-45.4° を確認できました。総合レポートをまとめます。 

Completed: *総合分析・レポート作成* (5/5)

---

## センサーモデル検証 総合レポート

全9つのCSVファイルを包括的に分析し、シミュレーションモデルとの整合性を検証しました。以下が結果です。

---

### 各CSVの実際の運動パターン

| ファイル | 検出された運動 | 確認結果 |
|---------|--------------|---------|
| **stillness.csv** | 水平静止 30s、中盤に1秒の外乱 | accel_z ≈ 9.67, Roll≈-1°, Pitch≈-1° ✅ |
| **Tilt right.csv** | 水平 → 右傾斜90° → 水平の繰り返し | Roll = 88.2° 確認 (accel_y ≈ 9.65) ✅ |
| **Upward.csv** | 水平 → 前方45°傾斜 → 水平 | Pitch = -45.4° 確認 ✅ |
| **Clockwise.csv** | 水平保持で時計回り1周 (4セグメント × 90°) | 総Yaw = -363.9° ≈ 360° ✅ |
| **Right rotation.csv** | X軸(Roll)周り回転 ≈ 2回転 | Roll支配的、総回転 774° ✅ |
| **Upward and downward.csv** | ピッチ方向の前後往復 (±76°) | Pitch支配的、周期 ≈ 2.8s ✅ |
| **One rotation left and right.csv** | 水平Yaw左回転1周 → 右回転1周 | 左-365° + 右+375° ≈ 原点復帰 ✅ |
| **Up and down.csv** | 上下並進 (手動で持ち上げ/下げ) | accel_z変動、周期 ≈ 5.1s |
| **Back and forth.csv** | 前後並進 (小さなYaw振動を含む) | accel_x正負ピーク確認 ✅ |

---

### 整合性: ✅ 確認済みの項目

1. **座標系 (Z-up)**: 加速度計Z軸が重力方向と一致。水平時 accel_z ≈ +9.67, Roll=90° 時 accel_y ≈ +9.65
2. **回転行列**: Tilt right (Roll=88.2°) と Upward (Pitch=-45.4°) で重力分解が物理的に整合
3. **ジャイロ積分**: Clockwise ≈ 360°, Right rotation ≈ 2回転, One rotation ≈ 左右各1回転
4. **ジャイロ-磁気計の一致**: Clockwise で磁気方位変化 365° vs ジャイロ積分 364° (差 1.6°)
5. **磁気円フィット**: ヨー回転時に mag_x/mag_y が円軌跡 (RMSE/R = 2.4%)
6. **ソフトアイアン歪み**: 無視可能 (軸比 0.97)
7. **サンプリングレート**: 100Hz で安定 (dt = 10ms)
8. **加速度単位**: m/s² ✅
9. **ジャイロ単位**: deg/s ✅

---

### 不一致: ❌ モデル修正が必要

#### 1. ジャイロバイアスモデル（最重要）

| 項目 | シミュレーション | 実機 |
|------|--------------|------|
| 初期バイアス | 0 deg/s | **30-65 deg/s** |
| セッション間変動 | アラン偏差 σ=0.5 | **σ > 13 deg/s** |
| 1セッション内安定性 | - | σ ≈ 0.05 deg/s (良好) |

シミュレーションではバイアス=0 でアラン偏差のみモデル化していますが、実機は 30-65 deg/s の巨大な固定バイアスを持ち、セッション（電源再投入）ごとに大きく変動します。フィルタの `bg` 状態で推定される設計ですが、シミュレーション上でもこのバイアスを再現しないとフィルタ性能の正確な評価ができません。

**推奨**: config_params.m に初期ジャイロバイアスパラメータを追加

#### 2. 磁場モデル（伏角なし）

| 項目 | シミュレーション | 実機 |
|------|--------------|------|
| mag_world | `[50, 0, 0]` (水平北のみ) | `[-38.6, -12.8, 6.8]` μT (伏角あり) |
| 伏角 | 0° | 実測 ≈ 9-17° (ハードアイアン影響), 日本理論値 ≈ 49° |
| ハードアイアン | なし | X: -18.6 μT, Y: 0.2 μT |

**推奨**: `mag_world = [Bh*cos(dec), Bh*sin(dec), -Bv]` に変更（日本: Bh≈30, Bv≈37, dec≈-8°）

#### 3. 磁気計の単位コメント誤り

generate_sensor_observations.m の `mag_strength = 50; % nT` は誤りで、実質 **μT** です（地球磁場 ≈ 50 μT。50 nT では地球磁場の1/1000）。

---

### 不一致: ⚠️ パラメータ調整推奨

#### 4. ノイズレベルの差異

| センサー | シミュレーション σ | 実測 σ | 比率 | 推奨 |
|---------|------------------|--------|------|------|
| **加速度** | 0.22 m/s² (white+pink) | 0.03 m/s² | 0.14x | `accel_pink_std` → 0.02 |
| **ジャイロ** | 0.54 deg/s (white+pink) | 0.45 deg/s | 0.83x | 概ねOK |
| **磁気** | 5.0 (μT?) | 0.41 μT | 0.08x | `mag_std` → 0.5 |
| **気圧** | 24 Pa (=2.0m) | 1.6 Pa (≈0.13m) | 0.07x | `baro_std` → 0.15 |

加速度と気圧のシミュレーションノイズが実機より**7-14倍大きい**。これはフィルタの共分散パラメータ (Q, R) の設定にも影響するため、実機値に合わせる調整が重要です。

#### 5. 加速度計スケールファクター

実機 |g| ≈ 9.67 m/s²（期待値 9.81 の -1.4%）。シミュレーションは理想的スケール(1.0)を想定。

#### 6. 気圧データ欠損

stillness.csv 以外の全ファイルで `baro = 0`。センサーの気圧データ取得に問題がある可能性があります。

#### 7. ジャイロY軸のPSD特性

ジャイロ X/Z 軸は log-log 傾き ≈ -3.7（ブラウンノイズ = アラン偏差の特徴）ですが、この傾きの急峻さはシミュレーションの `gyro_allan_std = 0.5` では再現できない可能性があります。

---

### 検証用スクリプト

分析に使用したスクリプトが comport/ フォルダに保存されています:
- analyze_sensor_data.py — 全CSV概要統計・運動フェーズ検出
- deep_analysis.py — 静止キャリブレーション・モデル詳細検証

Made changes.

## 変更履歴（追記）

- 単位の修正: 地磁気の単位を `nT` → `μT` にリポジトリ内で統一しました。
  - 修正済みファイル:
    - `kalman/GenerateData/generate_sensor_observations.m`  (コメント、単位注記)
    - `kalman/GenerateData/config_params.m`               (mag_std コメント/アウトライヤ範囲注記)
    - `docs/SENSOR_MODEL_VALIDATION_PLAN.md`             (ドキュメント表記)
    - `kalman/compare_real_sim_sensors.m`                (プロットラベル)
    - `kalman/generate_validation_sim.m`                 (プロットラベル・出力文字列)
    - `kalman/cpp/Lib/Sensor/sensor_processing.hpp`      (APIコメント単位)
    - `kalman/comport/analyze_sensor_data.py` 等         (解析メッセージの表記修正)

- 推奨: 実測磁気ノイズに合わせて `kalman/GenerateData/config_params.m` の
  `params.noise.base.mag_std` を約 `0.4-0.5` μT に更新することを提案します。

（この追記は自動修正の記録です。必要であれば設定ファイルの具体的な数値変更も行います。）