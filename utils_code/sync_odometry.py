import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
import tf.transformations as tft

def read_trajectory(file_path):
    df = pd.read_csv(file_path, sep=' ', header=None, names=['time', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
    return df

def linear_interpolate_positions(times, positions, target_times):
    interpolator = interp1d(times, positions, axis=0, kind='linear', fill_value='extrapolate')
    return interpolator(target_times)

def quaternion_to_euler(q):
    return tft.euler_from_quaternion(q)

def euler_to_quaternion(roll, pitch, yaw):
    return tft.quaternion_from_euler(roll, pitch, yaw)

def synchronize_trajectories(gt_df, dp_df, dr_df):
    gt_times = gt_df['time'].values
    dp_times = dp_df['time'].values
    dr_times = dr_df['time'].values

    dp_positions = dp_df[['tx', 'ty', 'tz']].values
    dr_positions = dr_df[['tx', 'ty', 'tz']].values
    dp_interp_positions = linear_interpolate_positions(dp_times, dp_positions, gt_times)
    dr_interp_positions = linear_interpolate_positions(dr_times, dr_positions, gt_times)

    dp_quaternions = dp_df[['qx', 'qy', 'qz', 'qw']].values
    dr_quaternions = dr_df[['qx', 'qy', 'qz', 'qw']].values

    dp_interp_quaternions = []
    dr_interp_quaternions = []

    for t_gt in gt_times:
        dp_index = np.searchsorted(dp_times, t_gt) - 1
        if dp_index < 0:
            dp_index = 0
        if dp_index >= len(dp_quaternions) - 1:
            dp_index = len(dp_quaternions) - 2

        t_prev_dp = dp_times[dp_index]
        t_next_dp = dp_times[dp_index + 1]
        t_ratio_dp = (t_gt - t_prev_dp) / (t_next_dp - t_prev_dp)
        q0_dp = dp_quaternions[dp_index]
        q1_dp = dp_quaternions[dp_index + 1]

        euler0_dp = quaternion_to_euler(q0_dp)
        euler1_dp = quaternion_to_euler(q1_dp)

        euler_interp = np.array(euler0_dp) + t_ratio_dp * (np.array(euler1_dp) - np.array(euler0_dp))

        dp_interp_quaternions.append(euler_to_quaternion(*euler_interp))
        
        dr_index = np.searchsorted(dr_times, t_gt) - 1
        if dr_index < 0:
            dr_index = 0
        if dr_index >= len(dr_quaternions) - 1:
            dr_index = len(dr_quaternions) - 2

        t_prev_dr = dr_times[dr_index]
        t_next_dr = dr_times[dr_index + 1]
        t_ratio_dr = (t_gt - t_prev_dr) / (t_next_dr - t_prev_dr)
        q0_dr = dr_quaternions[dr_index]
        q1_dr = dr_quaternions[dr_index + 1]

        euler0_dr = quaternion_to_euler(q0_dr)
        euler1_dr = quaternion_to_euler(q1_dr)

        euler_interp = np.array(euler0_dr) + t_ratio_dr * (np.array(euler1_dr) - np.array(euler0_dr))

        dr_interp_quaternions.append(euler_to_quaternion(*euler_interp))

    dp_interp_quaternions = np.array(dp_interp_quaternions)
    dr_interp_quaternions = np.array(dr_interp_quaternions)

    assert len(dp_interp_positions) == len(gt_times), f"DP-INS size mismatch: {len(dp_interp_positions)} != {len(gt_times)}"
    assert len(dr_interp_positions) == len(gt_times), f"DR size mismatch: {len(dr_interp_positions)} != {len(gt_times)}"
    assert len(dp_interp_quaternions) == len(gt_times), f"DP-INS quaternion size mismatch: {len(dp_interp_quaternions)} != {len(gt_times)}"
    assert len(dr_interp_quaternions) == len(gt_times), f"DR quaternion size mismatch: {len(dr_interp_quaternions)} != {len(gt_times)}"

    dp_sync = pd.DataFrame(np.column_stack([gt_times, dp_interp_positions, dp_interp_quaternions]),
                           columns=['time', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
    dr_sync = pd.DataFrame(np.column_stack([gt_times, dr_interp_positions, dr_interp_quaternions]),
                           columns=['time', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
    
    return dp_sync, dr_sync

def save_trajectory(df, file_path):
    df.to_csv(file_path, sep=' ', header=False, index=False, float_format='%.8f')

def main():
    gt_df = read_trajectory('1.txt')
    dp_df = read_trajectory('2.txt')
    dr_df = read_trajectory('3.txt')

    dp_sync, dr_sync = synchronize_trajectories(gt_df, dp_df, dr_df)

    save_trajectory(gt_df, '1_output.txt')
    save_trajectory(dp_sync, '2_output.txt')
    save_trajectory(dr_sync, '3_output.txt')

if __name__ == "__main__":
    main()