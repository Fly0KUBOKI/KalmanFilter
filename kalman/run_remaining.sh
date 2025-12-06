#!/bin/bash
cd /cygdrive/c/Users/takut/OneDrive/ドキュメント/MATLAB/KalmanFilter/kalman

for i in 5 6 7 8 9 10; do
    echo "=========================================="
    echo "Starting Run $i"
    echo "=========================================="
    
    matlab -batch "run_single_wrapper($i)" 2>&1 | tee run_$i.log
    
    if [ $? -eq 0 ]; then
        echo "Run $i completed successfully"
    else
        echo "Run $i FAILED"
    fi
done

echo "All runs complete!"
