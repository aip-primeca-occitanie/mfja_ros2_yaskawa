# Script to verify a sequence of trajectories

# The input are a number of trajectories N and a path
# To get the trajectory files, we append "{i}.csv" to this path input with i=1..N

# Each line contains is six values separated by commas, representing a trajectory point
# Each trajectory must have 200 or fewer points
# The last point of each trajectory must be the same as the first point of the following trajectory
# The values must be in radians and not degrees
# No header line with the joint names
#
# The script verify each criterion and displays for each if it is respected or not
import os
import csv
import math

def verify_trajectories(n: int, base_path: str):
    """
    Verifies a sequence of N trajectory CSV files based on specific criteria.
    
    Parameters:
    - n (int): Number of trajectories.
    - base_path (str): The prefix path where files are located (e.g., 'data/traj_').
    """
    all_passed = True
    last_point_prev = None
    
    print(f"=== Starting Verification for {n} Trajectories ===")
    print(f"Base Path Prefix: '{base_path}'\n")
    
    for i in range(1, n + 1):
        file_path = f"{base_path}{i}.csv"
        print(f"--- Checking Trajectory {i} ({file_path}) ---")
        
        # 1. Check if file exists
        if not os.path.exists(file_path):
            print(f"  [FAIL] File does not exist: {file_path}")
            all_passed = False
            continue
            
        points = []
        has_header = False
        format_valid = True
        
        try:
            with open(file_path, 'r', newline='') as f:
                reader = csv.reader(f)
                for row_idx, row in enumerate(reader):
                    if not row:
                        continue
                        
                    # Check for header line on the first row
                    if row_idx == 0:
                        try:
                            [float(val) for val in row]
                        except ValueError:
                            has_header = True
                            print(f"  [FAIL] Header line detected with joint names on row 1: {row}")
                            all_passed = False
                            continue
                            
                    # Check if each line has exactly 6 values
                    if len(row) != 6:
                        print(f"  [FAIL] Row {row_idx + 1} does not have exactly 6 values (found {len(row)}).")
                        format_valid = False
                        all_passed = False
                        break
                        
                    # Convert to float values
                    try:
                        point = [float(val) for val in row]
                        points.append(point)
                    except ValueError:
                        print(f"  [FAIL] Row {row_idx + 1} contains non-numeric values.")
                        format_valid = False
                        all_passed = False
                        break
        except Exception as e:
            print(f"  [FAIL] Error reading file: {e}")
            all_passed = False
            continue
            
        if not format_valid or has_header:
            print()
            continue
            
        # Criterion 2: Each trajectory must have 200 or fewer points
        num_points = len(points)
        if num_points <= 200:
            print(f"  [PASS] Point count: {num_points} points (<= 200 limit).")
        else:
            print(f"  [FAIL] Point count: {num_points} points (exceeds 200 limit).")
            all_passed = False
            
        # Criterion 3: Values must be in radians and not degrees
        # Heuristic: If any value has an absolute magnitude greater than 2*pi (~6.283), 
        # it is likely represented in degrees rather than radians.
        radians_valid = True
        for pt in points:
            for val in pt:
                if abs(val) > 2 * math.pi:
                    radians_valid = False
                    break
            if not radians_valid:
                break
                
        if radians_valid:
            print(f"  [PASS] Values appear to be in radians (all values within [-2*pi, 2*pi]).")
        else:
            print(f"  [FAIL] Values appear to be in degrees (found magnitude > 2*pi).")
            all_passed = False
            
        # Criterion 4: The last point of the previous trajectory must match the first point of the current one
        if last_point_prev is not None and len(points) > 0:
            current_first = points[0]
            tolerance = 1e-6  # Small tolerance for floating-point comparisons
            matches = all(abs(a - b) < tolerance for a, b in zip(last_point_prev, current_first))
            
            if matches:
                print(f"  [PASS] Seamless transition from trajectory {i-1} to {i}.")
            else:
                print(f"  [FAIL] Discontinuity between trajectory {i-1} end and trajectory {i} start.")
                print(f"         Previous last point: {last_point_prev}")
                print(f"         Current first point: {current_first}")
                all_passed = False
                
        # Store the last point of this trajectory for the next iteration
        if len(points) > 0:
            last_point_prev = points[-1]
            
        print()
        
    # Final Summary
    if all_passed:
        print("=== VERIFICATION SUCCESSFUL: All criteria respected! ===")
    else:
        print("=== VERIFICATION FAILED: One or more criteria were violated. ===")

if __name__ == "__main__":
    # Example configuration:
    # Set your number of trajectories (N) and file path prefix here
    N_TRAJECTORIES = 5
    PATH_PREFIX = "trajectory_"  # Will check trajectory_1.csv, trajectory_2.csv, etc.
    
    verify_trajectories(N_TRAJECTORIES, PATH_PREFIX)