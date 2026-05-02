import argparse
import glob
import os
import shutil
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation


METRIC_NAMES = [
    'E_trans_RMSE',
    'E_trans_MAE',
    'E_rot_RMSE',
    'E_rot_MAE',
    'RTE_p_RMSE',
    'RTE_p_MAE',
    'RTE_r_RMSE',
    'RTE_r_MAE',
    'CRS',
    'TCR',
]


def load_tum_file(filepath):
    data = np.loadtxt(filepath)
    if data.ndim == 1:
        if data.shape[0] != 8:
            raise ValueError(f'invalid TUM row in {filepath}')
        data = data.reshape(1, -1)
    if data.shape[1] != 8:
        raise ValueError(f'invalid TUM format in {filepath}')
    return data


def load_groundtruth(filepath):
    gt_data = load_tum_file(filepath)
    if len(gt_data) == 0:
        raise ValueError('ground-truth trajectory is empty')
    return gt_data


def load_estimated_segments(estimated_path):
    temp_dir = None
    folder = estimated_path

    if os.path.isfile(estimated_path):
        temp_dir = tempfile.TemporaryDirectory()
        folder = temp_dir.name
        shutil.copy2(estimated_path, os.path.join(folder, os.path.basename(estimated_path)))
    elif not os.path.isdir(estimated_path):
        raise FileNotFoundError(f'estimated path not found: {estimated_path}')

    files = sorted(
        f for f in glob.glob(os.path.join(folder, '*.txt'))
        if 'result' not in os.path.basename(f).lower()
    )
    if not files:
        if temp_dir is not None:
            temp_dir.cleanup()
        raise ValueError(f'no estimated .txt files found in {estimated_path}')

    segments = []
    merged = []
    for file in files:
        data = load_tum_file(file)
        if len(data) == 0:
            continue
        source = os.path.basename(file)
        segments.append((data, source))
        for pose in data:
            merged.append({
                'timestamp': float(pose[0]),
                'position': pose[1:4].astype(float),
                'quaternion': pose[4:8].astype(float),
                'source_file': source,
            })

    if not segments:
        if temp_dir is not None:
            temp_dir.cleanup()
        raise ValueError('no valid estimated trajectories loaded')

    segments.sort(key=lambda item: float(item[0][0, 0]))
    merged.sort(key=lambda item: item['timestamp'])
    return segments, merged, temp_dir


def detect_failures(est_data):
    failures = set()
    for i in range(1, len(est_data)):
        if est_data[i - 1]['source_file'] != est_data[i]['source_file']:
            failures.add(i)
    return failures


def align_timestamps(gt_segment, est_segment, max_diff=0.02):
    aligned_gt = []
    aligned_est = []

    gt_times = gt_segment[:, 0]
    est_times = np.array([e['timestamp'] for e in est_segment], dtype=float)

    for i, est_time in enumerate(est_times):
        time_diffs = np.abs(gt_times - est_time)
        min_idx = int(np.argmin(time_diffs))
        if time_diffs[min_idx] < max_diff:
            aligned_gt.append(gt_segment[min_idx])
            aligned_est.append(est_segment[i])

    return aligned_gt, aligned_est


def umeyama_alignment(src_points, dst_points):
    src = np.asarray(src_points, dtype=float).T
    dst = np.asarray(dst_points, dtype=float).T

    num_points = src.shape[1]
    src_mean = np.mean(src, axis=1, keepdims=True)
    dst_mean = np.mean(dst, axis=1, keepdims=True)

    src_centered = src - src_mean
    dst_centered = dst - dst_mean
    h_mat = src_centered @ dst_centered.T / num_points

    u_mat, _, vt_mat = np.linalg.svd(h_mat)
    r_mat = vt_mat.T @ u_mat.T
    if np.linalg.det(r_mat) < 0:
        vt_mat[-1, :] *= -1
        r_mat = vt_mat.T @ u_mat.T

    t_vec = dst_mean - r_mat @ src_mean
    return r_mat, t_vec.flatten()


def evaluate_segment(gt_segment, est_segment):
    result = {
        'e_t': None,
        'e_r': None,
        'A_t': 0.0,
        'A_r': 0.0,
        'F_i': 0,
        'aligned_gt': None,
        'aligned_est': None,
        'est_aligned_positions': None,
        'est_aligned_quats': None,
    }

    if len(gt_segment) == 0 or len(est_segment) == 0:
        return result

    aligned_gt, aligned_est = align_timestamps(gt_segment, est_segment)
    if len(aligned_gt) < 3:
        return result

    gt_positions = np.array([p[1:4] for p in aligned_gt], dtype=float)
    est_positions = np.array([e['position'] for e in aligned_est], dtype=float)
    est_quats = np.array([e['quaternion'] for e in aligned_est], dtype=float)

    r_trans, t_trans = umeyama_alignment(est_positions, gt_positions)
    est_aligned_positions = (r_trans @ est_positions.T).T + t_trans

    r_rot = Rotation.from_matrix(r_trans)
    est_aligned_quats = np.array([(r_rot * Rotation.from_quat(q)).as_quat() for q in est_quats])

    trans_errors = np.linalg.norm(est_aligned_positions - gt_positions, axis=1)
    e_t = float(np.sqrt(np.mean(trans_errors ** 2)))

    gt_rotations = [Rotation.from_quat(p[4:8]).as_matrix() for p in aligned_gt]
    rot_errors = []
    for i in range(len(aligned_gt)):
        est_rot_aligned = r_trans @ Rotation.from_quat(aligned_est[i]['quaternion']).as_matrix()
        rel_rot = Rotation.from_matrix(gt_rotations[i].T @ est_rot_aligned)
        rot_errors.append(rel_rot.magnitude())
    e_r = float(np.sqrt(np.mean(np.square(rot_errors))))

    result.update({
        'e_t': e_t,
        'e_r': e_r,
        'A_t': float(np.exp(-1.0 * e_t)),
        'A_r': float(np.exp(-0.5 * e_r)),
        'F_i': 1,
        'aligned_gt': aligned_gt,
        'aligned_est': aligned_est,
        'est_aligned_positions': est_aligned_positions,
        'est_aligned_quats': est_aligned_quats,
    })
    return result


def calculate_coverage_metrics(est_segments, gt_start_time, gt_end_time):
    total_time = gt_end_time - gt_start_time
    if total_time <= 0:
        return {'CRS': 0.0, 'TCR': 0.0}

    coverage_segments = []
    for segment_data, source_file in est_segments:
        seg_start = float(segment_data[0, 0])
        seg_end = float(segment_data[-1, 0])
        inter_start = max(seg_start, gt_start_time)
        inter_end = min(seg_end, gt_end_time)
        if inter_end > inter_start:
            coverage_segments.append({
                'source_file': source_file,
                'duration': inter_end - inter_start,
            })

    total_coverage_time = sum(seg['duration'] for seg in coverage_segments)
    max_continuous_time = max((seg['duration'] for seg in coverage_segments), default=0.0)

    crs = min(max(total_coverage_time / total_time, 0.0), 1.0)
    tcr = min(max(max_continuous_time / total_time, 0.0), 1.0)
    return {'CRS': crs, 'TCR': tcr}


def compute_window_rpe(aligned_gt, est_aligned_positions, est_aligned_quats):
    pairs = []
    if aligned_gt is None or est_aligned_positions is None or len(aligned_gt) < 2:
        return pairs

    for k in range(len(aligned_gt) - 1):
        gt_i = aligned_gt[k]
        gt_j = aligned_gt[k + 1]

        t_gt_i = gt_i[1:4]
        t_gt_j = gt_j[1:4]
        r_gt_i = Rotation.from_quat(gt_i[4:8]).as_matrix()
        r_gt_j = Rotation.from_quat(gt_j[4:8]).as_matrix()

        t_est_i = est_aligned_positions[k]
        t_est_j = est_aligned_positions[k + 1]
        r_est_i = Rotation.from_quat(est_aligned_quats[k]).as_matrix()
        r_est_j = Rotation.from_quat(est_aligned_quats[k + 1]).as_matrix()

        r_rel_gt = r_gt_i.T @ r_gt_j
        t_rel_gt = r_gt_i.T @ (t_gt_j - t_gt_i)
        r_rel_est = r_est_i.T @ r_est_j
        t_rel_est = r_est_i.T @ (t_est_j - t_est_i)

        dt = float(gt_j[0] - gt_i[0])
        if dt <= 0:
            continue

        trans_err = float(np.linalg.norm(t_rel_est - t_rel_gt))
        rel_rot = Rotation.from_matrix(r_rel_gt.T @ r_rel_est)
        rot_err = float(rel_rot.magnitude())
        pairs.append({'dt': dt, 'e_t': trans_err, 'e_r': rot_err})

    return pairs


def compute_metrics(groundtruth_file, estimated_path, segment_duration, failure_threshold):
    del failure_threshold

    gt_data = load_groundtruth(groundtruth_file)
    est_segments, est_data, temp_dir = load_estimated_segments(estimated_path)

    try:
        failures = detect_failures(est_data)
        gt_start_time = float(gt_data[0, 0])
        gt_end_time = float(gt_data[-1, 0])

        current_time = gt_start_time
        window_results = []

        while current_time < gt_end_time:
            window_end = min(current_time + segment_duration, gt_end_time)

            gt_mask = (gt_data[:, 0] >= current_time) & (gt_data[:, 0] < window_end)
            gt_window = gt_data[gt_mask]

            est_window = []
            est_indices = []
            for i, pose in enumerate(est_data):
                if current_time <= pose['timestamp'] < window_end:
                    est_window.append(pose)
                    est_indices.append(i)

            failure_positions = [idx for idx in est_indices if idx in failures]

            if len(est_window) == 0:
                window_results.append({'status': 0, 'rpe_pairs': [], 'subsegments': []})
                current_time = window_end
                continue

            if len(failure_positions) == 0:
                segment_eval = evaluate_segment(gt_window, est_window)
                rpe_pairs = compute_window_rpe(
                    segment_eval['aligned_gt'],
                    segment_eval['est_aligned_positions'],
                    segment_eval['est_aligned_quats'],
                ) if segment_eval['F_i'] == 1 else []

                window_results.append({
                    'status': segment_eval['F_i'],
                    'e_t': segment_eval['e_t'],
                    'e_r': segment_eval['e_r'],
                    'rpe_pairs': rpe_pairs,
                    'subsegments': [],
                })
                current_time = window_end
                continue

            subsegments = []
            last_idx = 0
            for fail_idx in sorted(failure_positions):
                rel_idx = est_indices.index(fail_idx)
                if rel_idx > last_idx:
                    subsegments.append(('normal', last_idx, rel_idx))
                subsegments.append(('failure', rel_idx, rel_idx))
                last_idx = rel_idx
            if last_idx < len(est_window):
                subsegments.append(('normal', last_idx, len(est_window)))

            subsegments_info = []
            has_success = False
            for seg_type, start_idx, end_idx in subsegments:
                if seg_type != 'normal':
                    subsegments_info.append({'type': 'failure'})
                    continue

                sub_est = est_window[start_idx:end_idx]
                if not sub_est:
                    continue

                sub_start_time = sub_est[0]['timestamp']
                sub_end_time = sub_est[-1]['timestamp']
                sub_gt_mask = (gt_data[:, 0] >= sub_start_time) & (gt_data[:, 0] <= sub_end_time)
                sub_gt = gt_data[sub_gt_mask]
                segment_eval = evaluate_segment(sub_gt, sub_est)

                segment_info = {
                    'type': 'normal',
                    'F_i': segment_eval['F_i'],
                    'e_t': segment_eval['e_t'],
                    'e_r': segment_eval['e_r'],
                }
                subsegments_info.append(segment_info)
                if segment_eval['F_i'] == 1:
                    has_success = True

            window_results.append({
                'status': 1 if has_success else 0,
                'e_t': None,
                'e_r': None,
                'rpe_pairs': [],
                'subsegments': subsegments_info,
            })
            current_time = window_end

        seg_et_vals = []
        seg_et_abs = []
        seg_er_vals = []
        seg_er_abs = []

        sum_dt_global = 0.0
        sum_dt_et2_global = 0.0
        sum_dt_abs_et_global = 0.0
        sum_dt_er2_global = 0.0
        sum_dt_abs_er_global = 0.0

        for window in window_results:
            for pair in window.get('rpe_pairs', []):
                dt = float(pair['dt'])
                et = float(pair['e_t'])
                er = float(pair['e_r'])

                sum_dt_global += dt
                sum_dt_et2_global += (et ** 2) * dt
                sum_dt_abs_et_global += abs(et) * dt
                sum_dt_er2_global += (er ** 2) * dt
                sum_dt_abs_er_global += abs(er) * dt

            used_any_subsegment = False
            for segment in window.get('subsegments', []):
                if segment.get('type') != 'normal' or segment.get('F_i') != 1:
                    continue
                used_any_subsegment = True

                et = segment.get('e_t')
                er = segment.get('e_r')
                if et is not None:
                    et = float(et)
                    seg_et_vals.append(et)
                    seg_et_abs.append(abs(et))
                if er is not None:
                    er = float(er)
                    seg_er_vals.append(er)
                    seg_er_abs.append(abs(er))

            if not used_any_subsegment and window.get('status') == 1:
                et = window.get('e_t')
                er = window.get('e_r')
                if et is not None:
                    et = float(et)
                    seg_et_vals.append(et)
                    seg_et_abs.append(abs(et))
                if er is not None:
                    er = float(er)
                    seg_er_vals.append(er)
                    seg_er_abs.append(abs(er))

        coverage = calculate_coverage_metrics(est_segments, gt_start_time, gt_end_time)

        return {
            'E_trans_RMSE': float(np.sqrt(np.mean(np.square(seg_et_vals)))) if seg_et_vals else None,
            'E_trans_MAE': float(np.mean(seg_et_abs)) if seg_et_abs else None,
            'E_rot_RMSE': float(np.sqrt(np.mean(np.square(seg_er_vals)))) if seg_er_vals else None,
            'E_rot_MAE': float(np.mean(seg_er_abs)) if seg_er_abs else None,
            'RTE_p_RMSE': float(np.sqrt(sum_dt_et2_global / sum_dt_global)) if sum_dt_global > 0 else None,
            'RTE_p_MAE': float(sum_dt_abs_et_global / sum_dt_global) if sum_dt_global > 0 else None,
            'RTE_r_RMSE': float(np.sqrt(sum_dt_er2_global / sum_dt_global)) if sum_dt_global > 0 else None,
            'RTE_r_MAE': float(sum_dt_abs_er_global / sum_dt_global) if sum_dt_global > 0 else None,
            'CRS': float(coverage['CRS']),
            'TCR': float(coverage['TCR']),
        }
    finally:
        if temp_dir is not None:
            temp_dir.cleanup()


def main():
    parser = argparse.ArgumentParser(
        description='Print only E/RTE/CRS/TCR metrics from one ground-truth trajectory and one estimated trajectory.'
    )
    parser.add_argument('-g', '--groundtruth', required=True, help='Ground-truth TUM trajectory file')
    parser.add_argument('-e', '--estimated', required=True, help='Estimated TUM trajectory file or folder')
    parser.add_argument('-d', '--duration', type=float, default=60.0, help='Segment duration in seconds')
    parser.add_argument('-t', '--threshold', type=float, default=0.5, help='Failure threshold in seconds')
    args = parser.parse_args()

    metrics = compute_metrics(
        groundtruth_file=args.groundtruth,
        estimated_path=args.estimated,
        segment_duration=args.duration,
        failure_threshold=args.threshold,
    )

    for name in METRIC_NAMES:
        value = metrics[name]
        if value is None:
            print(f'{name}=None')
        else:
            print(f'{name}={value:.10f}')


if __name__ == '__main__':
    main()
