import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from sklearn.neighbors import KDTree

def load_pcd(path):
    pcd = o3d.io.read_point_cloud(path)
    return np.asarray(pcd.points)

def compute_nne(points):
    tree = cKDTree(points)
    dist, _ = tree.query(points, k=2)
    return np.mean(dist[:, 1])

def compute_ac_com(gt, est, tau=0.1):
    tree_est = cKDTree(est)

    dist_gt2est, _ = tree_est.query(gt)

    inlier_mask = dist_gt2est < tau

    COM = np.sum(inlier_mask) / len(gt)

    if np.sum(inlier_mask) > 0:
        AC = np.mean(dist_gt2est[inlier_mask])
    else:
        AC = np.nan

    return AC, COM

def compute_cd(gt, est):
    tree_gt = cKDTree(gt)
    tree_est = cKDTree(est)

    dist_est2gt, _ = tree_gt.query(est)
    dist_gt2est, _ = tree_est.query(gt)

    return np.mean(dist_gt2est) + np.mean(dist_est2gt)

def compute_mme(points, k=20):
    tree = KDTree(points)
    lambda_min_list = []

    for i in range(len(points)):
        idx = tree.query([points[i]], k=k, return_distance=False)[0]
        neighbors = points[idx]

        cov = np.cov(neighbors.T)
        eigvals = np.linalg.eigvalsh(cov)
        lambda_min = np.min(eigvals)

        lambda_min_list.append(lambda_min)

    lambda_min_array = np.array(lambda_min_list)
    return -np.mean(np.log(lambda_min_array + 1e-12))

def evaluate(gt_path, est_path, tau=0.1, k=20):
    gt = load_pcd(gt_path)
    est = load_pcd(est_path)

    print("Points:", len(gt), len(est))

    nne = compute_nne(est)
    ac, com = compute_ac_com(gt, est, tau)
    cd = compute_cd(gt, est)
    mme = compute_mme(est, k)

    print("\n====== Results ======")
    print(f"NNE ↓ : {nne:.6f}")
    print(f"AC  ↓ : {ac:.6f}")
    print(f"COM ↑ : {com:.6f}")
    print(f"CD  ↓ : {cd:.6f}")
    print(f"MME ↓ : {mme:.6f}")
    print("=====================")

if __name__ == "__main__":
    evaluate("gt.pcd", "eva.pcd", tau=0.1, k=20)