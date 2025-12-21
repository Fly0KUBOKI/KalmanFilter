import csv
import sys
from pathlib import Path

matlab_file = Path('Results/estimation_matlab.csv')
mex_file = Path('Results/estimation_mex.csv')
threshold = 1e3

if not matlab_file.exists() or not mex_file.exists():
    print('Missing CSV files: ensure Results/estimation_matlab.csv and Results/estimation_mex.csv exist')
    sys.exit(2)

with matlab_file.open('r', encoding='utf-8') as f:
    r = csv.reader(f)
    header_m = next(r)
    data_m = [row for row in r]
with mex_file.open('r', encoding='utf-8') as f:
    r = csv.reader(f)
    header_x = next(r)
    data_x = [row for row in r]

# find index of time column
try:
    ti_m = header_m.index('time')
    ti_x = header_x.index('time')
except ValueError:
    # try 'Time' fallback
    try:
        ti_m = header_m.index('Time')
        ti_x = header_x.index('Time')
    except ValueError:
        print('No time column found in headers')
        sys.exit(3)

# build time->row mapping for mex
map_x = {}
for i,row in enumerate(data_x):
    try:
        t = float(row[ti_x])
    except:
        continue
    map_x.setdefault(round(t,6),[]).append((i,row))

# iterate matlab rows and compare where common time exists
cols_m = [h for h in header_m if h!='time']
col_indices_m = [i for i,h in enumerate(header_m) if h!='time']

for i,row in enumerate(data_m):
    try:
        t = float(row[ti_m])
    except:
        continue
    key = round(t,6)
    if key not in map_x:
        continue
    # pick first matching mex row
    j, rowx = map_x[key][0]
    # compute max abs diff over all non-time numeric columns that exist in both
    max_err = 0.0
    for im, hm in zip(col_indices_m, cols_m):
        # find corresponding column in mex by name
        if hm not in header_x:
            continue
        ix = header_x.index(hm)
        try:
            vm = float(row[im])
            vx = float(rowx[ix])
            err = abs(vx - vm)
            if err>max_err:
                max_err = err
        except:
            continue
    if max_err>threshold:
        print(f'First large diff at time={t} (matlab row {i+2}), max err={max_err}')
        print('matlab row (header->value):')
        for h, v in zip(header_m, row):
            print(f'  {h}: {v}')
        print('\nmex row (header->value):')
        for h, v in zip(header_x, rowx):
            print(f'  {h}: {v}')
        # suggest corresponding record file
        print(f"\nPossible record file: kalman/Results/record_runfilter_sample_{i+2}.mat or kalman/record_runfilter_sample_{i+2}.mat")
        sys.exit(0)

print('No sample exceeds threshold', threshold)
sys.exit(0)
