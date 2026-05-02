Eva traj.py

Eva traj.py is a single-file evaluation script used to compute metrics based on:

- Ground truth trajectory (GT)
- Estimated trajectory (EST)

It directly prints the following 10 metrics in the terminal:

- E_trans_RMSE
- E_trans_MAE
- E_rot_RMSE
- E_rot_MAE
- RTE_p_RMSE
- RTE_p_MAE
- RTE_r_RMSE
- RTE_r_MAE
- CRS
- TCR

It does not depend on other evaluation scripts in the repository and does not save any output files.

1. Install Dependencies

It is recommended to activate your Python environment first, then install:

pip install numpy scipy

Or using conda:

conda install numpy scipy

2. Input File Format

The script reads trajectories in TUM format, with 8 columns per line:

timestamp tx ty tz qx qy qz qw

Notes:

- GT must be a .txt file
- EST can be a .txt file
- EST can also be a folder, in which case all .txt trajectory files inside will be processed

3. How to Run

python "Eva traj.py" -g "GT_file.txt" -e "EST_file_or_folder"

Notes:

- Paths containing spaces must be quoted
- The script name contains a space, so it must also be quoted

4. Optional Arguments

The script supports two optional parameters:

-d, --duration
-t, --threshold

Example:

python "Eva traj.py" -g "GT_file.txt" -e "EST_file_or_folder" -d 60 -t 0.5

Meaning:

- -d: evaluate using time-window segmentation
- -t: failure threshold parameter (kept as an interface)

Default values:

- -d default is 60.0
- -t default is 0.5

5. Example Output

After running, the following will be printed:

E_trans_RMSE=...
E_trans_MAE=...
E_rot_RMSE=...
E_rot_MAE=...
RTE_p_RMSE=...
RTE_p_MAE=...
RTE_r_RMSE=...
RTE_r_MAE=...
CRS=...
TCR=...