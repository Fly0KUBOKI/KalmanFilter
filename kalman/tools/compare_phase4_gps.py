#!/usr/bin/env python3
import json
import math

def frob_norm(mat):
    s = 0.0
    for row in mat:
        for v in row:
            s += v*v
    return math.sqrt(s)

def to_matrix(flat):
    # assume length 9, row-major
    return [flat[0:3], flat[3:6], flat[6:9]]

with open('kalman/Results/phase4_gps_matlab_k_fixed.json','r',encoding='utf-8') as f:
    m = json.load(f)
with open('kalman/Results/phase4_gps_mex_analysis.json','r',encoding='utf-8') as f:
    mex = json.load(f)

mexm = mex.get('MEX', {})

res = {}
res['sample_matlab'] = m.get('sample_id')
res['sample_mex'] = mex.get('sample_id')

# innovation y
res['y_matlab'] = m.get('y')
res['y_mex'] = mexm.get('y')
res['y_diff'] = [ (m.get('y')[i] - mexm.get('y')[i]) for i in range(3) ]

# S matrices
S_matlab = m.get('S_matrix')
S_mex = to_matrix(mexm.get('S')) if mexm.get('S') else None
res['S_matlab_diag'] = [S_matlab[0][0], S_matlab[1][1], S_matlab[2][2]] if S_matlab else None
res['S_mex_diag'] = [S_mex[0][0], S_mex[1][1], S_mex[2][2]] if S_mex else None
res['S_diag_ratio'] = None
if S_matlab and S_mex:
    res['S_diag_ratio'] = [ (S_matlab[i][i] / S_mex[i][i]) if S_mex[i][i] != 0 else None for i in range(3) ]

# K_velocity
K_matlab = m.get('K_velocity')
K_mex = mexm.get('K_velocity')
res['K_matlab_frob'] = frob_norm(K_matlab)
res['K_mex_frob'] = frob_norm(K_mex)
# diff
K_diff = [[K_matlab[i][j] - K_mex[i][j] for j in range(3)] for i in range(3)]
res['K_diff_frob'] = frob_norm(K_diff)
res['K_rel_diff'] = res['K_diff_frob'] / res['K_matlab_frob'] if res['K_matlab_frob'] != 0 else None

# P_diag velocity
P_mex = mexm.get('P_diag')
P_mex_vel = [P_mex[3], P_mex[4], P_mex[5]] if P_mex else None
res['P_matlab_vel'] = m.get('P_diag_velocity')
res['P_mex_vel'] = P_mex_vel
res['P_vel_ratio'] = None
if res['P_matlab_vel'] and P_mex_vel:
    res['P_vel_ratio'] = [ (res['P_matlab_vel'][i] / P_mex_vel[i]) if P_mex_vel[i] != 0 else None for i in range(3) ]

with open('kalman/Results/phase4_gps_side_by_side_comparison.json','w',encoding='utf-8') as f:
    json.dump(res, f, indent=2)

print('Comparison written to kalman/Results/phase4_gps_side_by_side_comparison.json')
