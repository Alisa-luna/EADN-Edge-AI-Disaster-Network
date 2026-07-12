# ============================================
# full_evaluation.py
# 完整评估：双编码器 vs 物理公式 vs 线性基线
# ============================================

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from scipy.stats import pearsonr
from sklearn.linear_model import LinearRegression
from sklearn.metrics import r2_score, mean_absolute_error
import matplotlib.pyplot as plt
import h5py
from tqdm import tqdm
from collections import defaultdict
import pickle


# ==================== 模型定义 ====================

class SpectralEncoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv1d(1, 16, 3, padding=1)
        self.conv2 = nn.Conv1d(16, 32, 3, padding=1)
        self.conv3 = nn.Conv1d(32, 32, 3, padding=1)
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.head = nn.Sequential(
            nn.Flatten(), nn.Linear(32, 32), nn.ReLU(), nn.Linear(32, 1)
        )

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = self.pool(x)
        return self.head(x).squeeze(1)


class TemporalEncoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv1d(1, 16, 7, padding=3)
        self.conv2 = nn.Conv1d(16, 32, 7, padding=3)
        self.conv3 = nn.Conv1d(32, 32, 5, padding=2)
        self.pool = nn.AdaptiveAvgPool1d(1)
        self.head = nn.Sequential(
            nn.Flatten(), nn.Linear(32, 32), nn.ReLU(), nn.Linear(32, 1)
        )

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = self.pool(x)
        return self.head(x).squeeze(1)


class DualEncoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.tau_encoder = SpectralEncoder()
        self.sp_encoder = TemporalEncoder()

    def forward(self, x):
        return self.tau_encoder(x), self.sp_encoder(x)


# ==================== 混合预测系统 ====================

class HybridPredictor:
    """
    混合系统：
    - 物理公式：初动振幅 → S波峰值（文献系数）
    - 神经网络：初动波形 → τc + S-P时间
    """

    def __init__(self, model_path="dual_encoder.pth"):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = DualEncoder().to(self.device)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.eval()

    def predict(self, onset_norm, onset_peak_raw):
        """
        onset_norm: 归一化初动 (N, 1, 30)
        onset_peak_raw: 原始初动峰值 (N,)
        返回: dict
        """
        x = torch.from_numpy(onset_norm).float().to(self.device)
        with torch.no_grad():
            pred_tau, pred_sp = self.model(x)

        # 物理公式：S波峰值
        PHYSICAL_RATIO = 2.5
        full_pd_est = onset_peak_raw * PHYSICAL_RATIO
        pred_s = 0.5 + 0.9 * np.log10(full_pd_est + 1e-10)

        return {
            's_peak_log': pred_s,
            'tau_c_log': pred_tau.cpu().numpy(),
            'sp_time': pred_sp.cpu().numpy(),
        }


# ==================== 数据加载 ====================

def load_test_data(hdf5_path="test.hdf5", max_samples=50000):
    with h5py.File(hdf5_path, 'r') as f:
        traces = f['traces']
        p_arr = f['p_arrival'][:]
        s_arr = f['s_arrival'][:]

        data = {
            'onset_norm': [], 'onset_peak_raw': [],
            'true_s': [], 'true_tau': [], 'true_sp': [],
            'magnitude': [], 'station_id': [],
        }

        for i in tqdm(range(min(len(traces), max_samples)), desc="加载测试数据"):
            z = traces[i, :, 2].astype(np.float32)
            p_idx = int(p_arr[i])
            s_idx = int(s_arr[i])

            if p_idx < 50 or p_idx + 100 > len(z):
                continue

            onset = z[p_idx:p_idx + 30]
            full_P = z[p_idx:p_idx + 100]

            # 原始初动峰值
            onset_peak_raw = np.max(np.abs(onset))

            # 归一化初动
            onset_max = np.max(np.abs(onset)) + 1e-10
            onset_norm = onset / onset_max

            # τc
            num = np.sum(full_P ** 2)
            den = np.sum(np.diff(full_P) ** 2) * 100
            tau_c = 2 * np.pi * np.sqrt(num / (den + 1e-10)) if den > 1e-10 else 0

            # S-P时间
            sp = (s_idx - p_idx) / 100.0

            # S波峰值
            s_wave = z[p_idx + 50:p_idx + 350]
            if len(s_wave) < 100:
                continue
            s_peak = np.max(np.abs(s_wave))

            data['onset_norm'].append(onset_norm)
            data['onset_peak_raw'].append(onset_peak_raw)
            data['true_s'].append(np.log10(s_peak + 1e-10))
            data['true_tau'].append(np.log10(tau_c + 1e-10))
            data['true_sp'].append(sp)

            # 用S波峰值近似震级
            data['magnitude'].append(np.log10(s_peak + 1e-10))
            data['station_id'].append(i % 100)  # 简化：用索引代替台站

        return {k: np.array(v) for k, v in data.items()}


# ==================== 全面评估 ====================

def full_evaluation():
    print("=" * 70)
    print("📊 混合系统全面评估")
    print("=" * 70)

    # 加载数据
    data = load_test_data(max_samples=3000)

    # 划分训练/验证（只在验证集上评估）
    n = len(data['onset_norm'])
    idx = np.random.permutation(n)
    split = int(n * 0.5)
    val_idx = idx[split:]

    val_data = {k: v[val_idx] for k, v in data.items()}

    # 预测
    predictor = HybridPredictor("dual_encoder.pth")
    onset_norm = val_data['onset_norm'].reshape(-1, 1, 30)
    onset_raw = val_data['onset_peak_raw']

    pred = predictor.predict(onset_norm, onset_raw)

    true_s = val_data['true_s']
    true_tau = val_data['true_tau']
    true_sp = val_data['true_sp']
    magnitude = val_data['magnitude']

    # ====== 1. 整体指标 ======
    print(f"\n{'─' * 70}")
    print(f"📊 表1：整体预测性能")
    print(f"{'─' * 70}")
    print(f"{'指标':<15} {'方法':<18} {'r':<10} {'MAE':<10} {'R²':<10}")
    print(f"{'─' * 63}")

    for name, pred_vals, true_vals in [
        ('S波峰值(物理)', pred['s_peak_log'], true_s),
        ('τc(神经网络)', pred['tau_c_log'], true_tau),
        ('S-P(神经网络)', pred['sp_time'], true_sp),
    ]:
        r, _ = pearsonr(pred_vals, true_vals)
        mae = np.mean(np.abs(pred_vals - true_vals))
        r2 = r2_score(true_vals, pred_vals)
        print(f"{name:<15} {'混合系统':<18} {r:.4f}     {mae:.4f}     {r2:.4f}")

    # 线性基线
    onset_raw_log = np.log10(val_data['onset_peak_raw'] + 1e-10)
    lr_s = LinearRegression().fit(onset_raw_log.reshape(-1, 1), true_s)
    pred_s_linear = lr_s.predict(onset_raw_log.reshape(-1, 1))
    r_s_linear, _ = pearsonr(pred_s_linear, true_s)
    mae_s_linear = np.mean(np.abs(pred_s_linear - true_s))
    print(
        f"{'S波峰值(线性)':<15} {'初动→S波':<18} {r_s_linear:.4f}     {mae_s_linear:.4f}     {r2_score(true_s, pred_s_linear):.4f}")

    # τc均值基线
    pred_tau_mean = np.full_like(true_tau, np.mean(true_tau))
    r_tau_mean, _ = pearsonr(pred_tau_mean, true_tau)
    print(f"{'τc(猜均值)':<15} {'基线':<18} {r_tau_mean:.4f}     --          --")

    # ====== 2. 分震级评估 ======
    print(f"\n{'─' * 70}")
    print(f"📊 表2：分震级（S波峰值）性能")
    print(f"{'─' * 70}")

    mag_bins = np.percentile(magnitude, [0, 25, 50, 75, 100])
    print(f"{'震级范围(logS)':<20} {'样本':<8} {'物理公式r':<12} {'线性基线r':<12} {'提升':<10}")
    print(f"{'─' * 62}")

    for i in range(len(mag_bins) - 1):
        mask = (magnitude >= mag_bins[i]) & (magnitude < mag_bins[i + 1])
        if mask.sum() < 10:
            continue

        r_phys, _ = pearsonr(pred['s_peak_log'][mask], true_s[mask])
        r_lin, _ = pearsonr(pred_s_linear[mask], true_s[mask])

        print(f"{'M' + str(round(mag_bins[i], 1)) + '~' + str(round(mag_bins[i + 1], 1)):<20} "
              f"{mask.sum():<8} {r_phys:.4f}       {r_lin:.4f}       {r_phys - r_lin:+.4f}")

    # ====== 3. 分S-P时间评估 ======
    print(f"\n{'─' * 70}")
    print(f"📊 表3：分S-P时间性能")
    print(f"{'─' * 70}")

    sp_bins = [0, 2, 5, 10, 20, 50]
    print(f"{'S-P时间(s)':<15} {'样本':<8} {'τc_r':<10} {'SP_r':<10} {'物理S_r':<10}")
    print(f"{'─' * 53}")

    for i in range(len(sp_bins) - 1):
        mask = (true_sp >= sp_bins[i]) & (true_sp < sp_bins[i + 1])
        if mask.sum() < 10:
            continue

        r_tau, _ = pearsonr(pred['tau_c_log'][mask], true_tau[mask])
        r_sp, _ = pearsonr(pred['sp_time'][mask], true_sp[mask])
        r_s, _ = pearsonr(pred['s_peak_log'][mask], true_s[mask])

        print(f"{sp_bins[i]}-{sp_bins[i + 1]:<10} {mask.sum():<8} "
              f"{r_tau:.4f}     {r_sp:.4f}     {r_s:.4f}")

    # ====== 4. 预测 vs 真实散点图 ======
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    for ax, pred_vals, true_vals, name, color in [
        (axes[0, 0], pred['s_peak_log'], true_s, 'S波峰值(物理公式)', '#2196F3'),
        (axes[0, 1], pred_s_linear, true_s, 'S波峰值(线性基线)', '#FF9800'),
        (axes[0, 2], pred['tau_c_log'], true_tau, 'τc(神经网络)', '#4CAF50'),
        (axes[1, 0], pred['sp_time'], true_sp, 'S-P时间(神经网络)', '#E91E63'),
    ]:
        r, _ = pearsonr(pred_vals, true_vals)
        ax.scatter(true_vals, pred_vals, alpha=0.3, s=10, c=color)
        ax.plot([true_vals.min(), true_vals.max()],
                [true_vals.min(), true_vals.max()], 'r--', linewidth=1)
        ax.set_xlabel('True')
        ax.set_ylabel('Pred')
        ax.set_title(f'{name}\nr={r:.3f}')

    # 残差分布
    residuals_s = pred['s_peak_log'] - true_s
    residuals_tau = pred['tau_c_log'] - true_tau
    residuals_sp = pred['sp_time'] - true_sp

    axes[1, 1].hist(residuals_s, bins=40, alpha=0.5, label='S波峰值', color='#2196F3')
    axes[1, 1].hist(residuals_tau, bins=40, alpha=0.5, label='τc', color='#4CAF50')
    axes[1, 1].axvline(x=0, color='red', linestyle='--')
    axes[1, 1].set_xlabel('Residual')
    axes[1, 1].set_title('残差分布')
    axes[1, 1].legend()

    # 分震级柱状图
    mag_bins = np.percentile(magnitude, [0, 20, 40, 60, 80, 100])
    r_phys_per_mag = []
    r_lin_per_mag = []
    labels = []
    for i in range(len(mag_bins) - 1):
        mask = (magnitude >= mag_bins[i]) & (magnitude < mag_bins[i + 1])
        if mask.sum() < 10: continue
        r_phys_per_mag.append(pearsonr(pred['s_peak_log'][mask], true_s[mask])[0])
        r_lin_per_mag.append(pearsonr(pred_s_linear[mask], true_s[mask])[0])
        labels.append(f'{mag_bins[i]:.1f}')

    x = np.arange(len(labels))
    axes[1, 2].bar(x - 0.15, r_phys_per_mag, 0.3, alpha=0.7, label='物理公式')
    axes[1, 2].bar(x + 0.15, r_lin_per_mag, 0.3, alpha=0.7, label='线性基线')
    axes[1, 2].set_xticks(x)
    axes[1, 2].set_xticklabels(labels, rotation=45)
    axes[1, 2].set_ylabel('Pearson r')
    axes[1, 2].set_title('分震级S波峰值预测')
    axes[1, 2].legend()

    plt.tight_layout()
    plt.savefig("full_evaluation_report.png", dpi=150)
    plt.show()

    # ====== 5. 总结 ======
    print(f"\n{'=' * 70}")
    print(f"📊 总结")
    print(f"{'=' * 70}")

    improvements = {
        'S波峰值': (r2_score(true_s, pred['s_peak_log']) - r2_score(true_s, pred_s_linear)),
        'τc': r2_score(true_tau, pred['tau_c_log']),
        'S-P时间': r2_score(true_sp, pred['sp_time']),
    }

    for name, imp in improvements.items():
        if imp > 0.05:
            print(f"  ✅ {name}: R²提升 {imp:+.3f}")
        elif imp > 0:
            print(f"  ⚠️ {name}: R²微弱提升 {imp:+.3f}")
        else:
            print(f"  ❌ {name}: 未超过基线")

    print(f"\n模型文件: dual_encoder.pth")
    print(f"报告图片: full_evaluation_report.png")

    return pred, val_data


if __name__ == "__main__":
    full_evaluation()