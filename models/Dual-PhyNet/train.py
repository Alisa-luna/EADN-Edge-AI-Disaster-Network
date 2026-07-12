# ============================================
# dual_encoder.py
# 双独立编码器：频谱编码器→τc，时序编码器→S-P
# ============================================

import h5py
import numpy as np
from scipy.stats import pearsonr
import torch
import torch.nn as nn
import torch.nn.functional as F
import matplotlib.pyplot as plt
from tqdm import tqdm


class SpectralEncoder(nn.Module):
    """频谱编码器：只学频率特征 → τc"""

    def __init__(self):
        super().__init__()
        # 小卷积核抓高频细节
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
    """时序编码器：只学时间结构 → S-P"""

    def __init__(self):
        super().__init__()
        # 大卷积核抓长程时序
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
    """双独立编码器：不共享任何参数"""

    def __init__(self):
        super().__init__()
        self.tau_encoder = SpectralEncoder()
        self.sp_encoder = TemporalEncoder()

    def forward(self, x):
        return self.tau_encoder(x), self.sp_encoder(x)


def load_data(hdf5_path="train.hdf5", max_samples=30000):
    with h5py.File(hdf5_path, 'r') as f:
        traces = f['traces']
        p_arr = f['p_arrival'][:]
        s_arr = f['s_arrival'][:]

        X, y_tau, y_sp = [], [], []

        for i in tqdm(range(min(len(traces), max_samples)), desc="加载"):
            z = traces[i, :, 2].astype(np.float32)
            p_idx = int(p_arr[i])
            s_idx = int(s_arr[i])

            if p_idx < 50 or p_idx + 100 > len(z):
                continue

            onset = z[p_idx:p_idx + 30]
            full_P = z[p_idx:p_idx + 100]

            num = np.sum(full_P ** 2)
            den = np.sum(np.diff(full_P) ** 2) * 100
            tau_c = 2 * np.pi * np.sqrt(num / (den + 1e-10)) if den > 1e-10 else 0

            sp = (s_idx - p_idx) / 100.0

            onset_max = np.max(np.abs(onset)) + 1e-10
            X.append(onset / onset_max)
            y_tau.append(np.log10(tau_c + 1e-10))
            y_sp.append(sp)

        return (np.array(X).reshape(-1, 1, 30),
                np.array(y_tau), np.array(y_sp))


def train_dual(epochs=150, batch_size=64):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    X, y_tau, y_sp = load_data(max_samples=50000)

    n = len(X)
    idx = np.random.permutation(n)
    split = int(n * 0.85)
    train_idx, val_idx = idx[:split], idx[split:]

    X_train, X_val = X[train_idx], X[val_idx]
    y_tau_train, y_tau_val = y_tau[train_idx], y_tau[val_idx]
    y_sp_train, y_sp_val = y_sp[train_idx], y_sp[val_idx]

    print(f"训练: {len(X_train)}, 验证: {len(X_val)}")

    model = DualEncoder().to(device)
    print(f"τc编码器参数量: {sum(p.numel() for p in model.tau_encoder.parameters()):,}")
    print(f"S-P编码器参数量: {sum(p.numel() for p in model.sp_encoder.parameters()):,}")

    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)

    best = {'tau': 0, 'sp': 0}

    for epoch in range(epochs):
        model.train()
        perm = np.random.permutation(len(X_train))
        for i in range(0, len(perm), batch_size):
            idx_b = perm[i:i + batch_size]
            x = torch.from_numpy(X_train[idx_b]).float().to(device)

            pred_tau, pred_sp = model(x)

            loss_tau = F.mse_loss(pred_tau, torch.from_numpy(y_tau_train[idx_b]).float().to(device))
            loss_sp = F.mse_loss(pred_sp, torch.from_numpy(y_sp_train[idx_b]).float().to(device))

            # 两个任务完全独立，损失直接相加
            loss = loss_tau + loss_sp
            opt.zero_grad();
            loss.backward();
            opt.step()

        sch.step()

        # 验证
        model.eval()
        with torch.no_grad():
            x_val = torch.from_numpy(X_val).float().to(device)
            pred_tau, pred_sp = model(x_val)

            r_tau, _ = pearsonr(pred_tau.cpu().numpy(), y_tau_val)
            r_sp, _ = pearsonr(pred_sp.cpu().numpy(), y_sp_val)

        if r_tau > best['tau']: best['tau'] = r_tau
        if r_sp > best['sp']: best['sp'] = r_sp

        if (epoch + 1) % 20 == 0:
            print(f"Epoch {epoch + 1}: τc_r={r_tau:.3f}  SP_r={r_sp:.3f}  "
                  f"Best(τc={best['tau']:.3f}, SP={best['sp']:.3f})")

    torch.save(model.state_dict(), "dual_encoder.pth")

    print(f"\n{'=' * 60}")
    print(f"📊 双编码器结果")
    print(f"{'=' * 60}")
    print(f"τc (频谱编码器):      r = {best['tau']:.4f}")
    print(f"S-P (时序编码器):     r = {best['sp']:.4f}")
    print(f"模型已保存: dual_encoder.pth")

    return model


if __name__ == "__main__":
    train_dual()