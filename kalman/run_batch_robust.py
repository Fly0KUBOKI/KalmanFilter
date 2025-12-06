import subprocess
import os
import sys
import time
import csv
import glob

def run_batch():
    runs = list(range(1, 11))  # 10 runs
    
    results_dir = "Results"
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
        
    # Run simulations
    for run_idx in runs:
        summary_file = os.path.join(results_dir, f"summary_run{run_idx:02d}.csv")
        if os.path.exists(summary_file):
            print(f"Run {run_idx} already completed. Skipping.")
            continue

        print(f"========================================")
        print(f"Starting Run {run_idx}")
        print(f"========================================")
        
        log_file = f"run_{run_idx}.log"
        cmd = f'matlab -batch "run_single_wrapper({run_idx})"'
        
        with open(log_file, "w") as f:
            start_time = time.time()
            try:
                process = subprocess.run(cmd, shell=True, stdout=f, stderr=subprocess.STDOUT)
                duration = time.time() - start_time
                print(f"Run {run_idx} finished in {duration:.2f}s with exit code {process.returncode}")
                
                if process.returncode != 0:
                    print(f"ERROR: Run {run_idx} failed! Check {log_file}")
                else:
                    print(f"Run {run_idx} success.")
                    
            except Exception as e:
                print(f"Exception running Run {run_idx}: {e}")
        
        print("\n")

    # Aggregate Results
    print("========================================")
    print("Aggregating Results...")
    print("========================================")
    
    summary_files = glob.glob(os.path.join(results_dir, "summary_run*.csv"))
    all_results = []
    
    for f in summary_files:
        try:
            with open(f, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    all_results.append(row)
        except Exception as e:
            print(f"Error reading {f}: {e}")

    # Sort by run_idx
    all_results.sort(key=lambda x: int(x.get('run_idx', 0)))
    
    # Print Summary Table
    if all_results:
        print(f"{'Run':<5} {'Pos RMSE':<10} {'Vel RMSE':<10} {'Att RMSE':<10} {'Status':<10}")
        print("-" * 50)
        for res in all_results:
            run_id = res.get('run_idx', '?')
            pos = float(res.get('pos_rmse', 0))
            vel = float(res.get('vel_rmse', 0))
            att = float(res.get('att_rmse', 0))
            diverged = res.get('diverged', '0')
            status = "FAIL" if diverged == '1' or diverged == 'true' else "OK"
            
            print(f"{run_id:<5} {pos:<10.4f} {vel:<10.4f} {att:<10.4f} {status:<10}")
            
        # Save combined CSV
        keys = all_results[0].keys()
        with open(os.path.join(results_dir, 'batch_summary_final.csv'), 'w', newline='') as output_file:
            dict_writer = csv.DictWriter(output_file, keys)
            dict_writer.writeheader()
            dict_writer.writerows(all_results)
        print(f"\nFinal summary saved to {os.path.join(results_dir, 'batch_summary_final.csv')}")
    else:
        print("No results found.")

if __name__ == "__main__":
    run_batch()
