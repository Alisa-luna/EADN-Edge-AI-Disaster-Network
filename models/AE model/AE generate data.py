import numpy as np


def moving_rms(sig, window=10):
    """滑动窗口 RMS 包络"""
    rms = np.zeros_like(sig)
    half = window // 2
    for t in range(len(sig)):
        start = max(0, t - half)
        end = min(len(sig), t + half + 1)
        rms[t] = np.sqrt(np.mean(sig[start:end] ** 2))
    return rms


def generate_normal_6ch(num_windows=50000, window_size=200):
    """生成 6 通道正常环境数据"""
    data = np.zeros((num_windows, 6, window_size))

    for i in range(num_windows):
        # 模拟不同时段的活动水平
        hour = np.random.randint(0, 24)
        if hour < 5:
            activity = 0.2  # 凌晨安静
        elif 7 <= hour <= 9 or 17 <= hour <= 19:
            activity = 1.0  # 早晚高峰
        else:
            activity = 0.6  # 日常

        # 基线 Z=1g，噪声水平受 activity 影响
        z = 1.0 + np.random.normal(0, 0.001 * activity, window_size)
        n = np.random.normal(0, 0.002 * activity, window_size)
        e = np.random.normal(0, 0.002 * activity, window_size)

        # 加低频环境振动
        z += 0.003 * activity * np.sin(2 * np.pi * np.random.uniform(0.1, 0.5) * np.arange(window_size) / 100)
        n += 0.002 * activity * np.sin(2 * np.pi * np.random.uniform(0.1, 0.8) * np.arange(window_size) / 100)
        e += 0.002 * activity * np.cos(2 * np.pi * np.random.uniform(0.1, 0.8) * np.arange(window_size) / 100)

        # 包络
        z_env = moving_rms(z, 10)
        n_env = moving_rms(n, 10)
        e_env = moving_rms(e, 10)

        data[i, 0] = z
        data[i, 1] = n
        data[i, 2] = e
        data[i, 3] = z_env
        data[i, 4] = n_env
        data[i, 5] = e_env

    # 窗口内逐通道归一化
    for i in range(num_windows):
        for c in range(6):
            ch = data[i, c]
            ch_max = np.max(np.abs(ch))
            if ch_max > 1e-10:
                data[i, c] = ch / (ch_max + 1e-10)

    return data


def generate_test_data(num_normal=500, num_anomaly=50):
    """生成测试数据（正常 + 异常）"""
    normal = generate_normal_6ch(num_normal)

    # 生成异常样本（模拟地震：Z 轴突增）
    anomaly = generate_normal_6ch(num_anomaly)
    for i in range(num_anomaly):
        # 在窗口后半段注入 P 波
        onset = np.random.randint(100, 140)
        amp = np.random.uniform(0.3, 1.0)
        p_wave = amp * np.sin(2 * np.pi * 2 * np.arange(60) / 100) * np.exp(-np.arange(60) / 20)
        anomaly[i, 0, onset:onset + 60] += p_wave  # Z 轴
        anomaly[i, 1, onset:onset + 60] += p_wave * np.random.uniform(0.2, 0.4)  # N 轴
        anomaly[i, 2, onset:onset + 60] += p_wave * np.random.uniform(0.2, 0.4)  # E 轴

        # 重新计算包络
        anomaly[i, 3] = moving_rms(anomaly[i, 0], 10)
        anomaly[i, 4] = moving_rms(anomaly[i, 1], 10)
        anomaly[i, 5] = moving_rms(anomaly[i, 2], 10)

        # 重新归一化
        for c in range(6):
            ch = anomaly[i, c]
            ch_max = np.max(np.abs(ch))
            if ch_max > 1e-10:
                anomaly[i, c] = ch / (ch_max + 1e-10)

    return normal, anomaly


if __name__ == '__main__':
    print("生成训练数据...")
    train_data = generate_normal_6ch(50000)
    np.save('normal_6ch.npy', train_data)
    print(f"训练数据: {train_data.shape}")

    print("生成测试数据...")
    normal_test, anomaly_test = generate_test_data(500, 50)
    np.save('test_normal.npy', normal_test)
    np.save('test_anomaly.npy', anomaly_test)
    print(f"正常测试: {normal_test.shape}, 异常测试: {anomaly_test.shape}")

    # 打印一个异常样本的 Z 轴
    print("异常样本 Z 轴前10点:", anomaly_test[0, 0, :10])