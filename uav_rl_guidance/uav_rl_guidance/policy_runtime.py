"""策略运行时：TorchScript 加载 + 特征构造 + 动作解码 + watchdog

依赖训练工程 guidance_rl（特征/几何/PNG 的唯一实现，避免两份代码漂移）：
  pip install -e /home/verser/Python/guidance_rl

watchdog 触发条件（任一）→ 本帧改用 PNG 回退指令并告警：
  - 策略输出含 NaN/Inf
  - 解码后速度指令模长超过 speed_cmd 的 1.5 倍
  - 推理抛异常
触发后连续 WATCHDOG_LATCH 帧内保持回退（防抖），期间策略隐藏状态保留。
"""
import json
import math
import os

import numpy as np

try:
    import torch
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "未找到 torch。部署环境需安装 PyTorch（CPU 版即可）: pip install torch"
    ) from e

try:
    from guidance_rl.features import FeatureBuilder, decode_action, FEATURE_VERSION
    from guidance_rl.png_teacher import PNGTeacher
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "未找到 guidance_rl 包（特征/PNG 的唯一实现）。"
        "请先安装: pip install -e /home/verser/Python/guidance_rl"
    ) from e

WATCHDOG_LATCH = 20   # 触发后保持回退的帧数（20Hz → 1s）


class PolicyRuntime:
    def __init__(self, model_path: str, png_fallback: PNGTeacher,
                 logger=None):
        """model_path: TorchScript 文件；同目录需有 *_meta.json"""
        self.logger = logger
        self.png = png_fallback

        meta_path = os.path.splitext(model_path)[0] + "_meta.json"
        with open(meta_path) as f:
            self.meta = json.load(f)
        if self.meta["feature_version"] != FEATURE_VERSION:
            raise RuntimeError(
                f"特征版本不匹配: 模型={self.meta['feature_version']} "
                f"运行时={FEATURE_VERSION}，请用当前 guidance_rl 重新导出模型")

        self.model = torch.jit.load(model_path, map_location="cpu")
        self.model.eval()

        cam = self.meta["camera"]
        self.fb = FeatureBuilder(cam["focal_length"],
                                 cam["image_width"], cam["image_height"])
        self._decode_kw = dict(self.meta["action_decode"])
        self._speed_limit = 1.5 * self._decode_kw["speed_cmd"]

        self.h = torch.zeros(1, 1, self.meta["gru_hidden"])
        self.watchdog_count = 0
        self.watchdog_total = 0

        # 预热（首帧推理 JIT 编译延迟）
        with torch.no_grad():
            self.model(torch.zeros(1, self.meta["obs_dim"]), self.h)

    def reset(self):
        """进入 INTERCEPT 时调用：清隐藏状态/特征缓存/回退状态"""
        self.h = torch.zeros(1, 1, self.meta["gru_hidden"])
        self.fb.reset()
        self.png.reset()
        self.watchdog_count = 0

    def step(self, det, roll, pitch, yaw, vx, vy, vz, local_z, dt):
        """单步制导（20Hz）

        det: (x, y, w, h) 或 None
        返回 (vx_cmd, vy_cmd, vz_cmd, yaw_rate, source)
          source ∈ {"policy", "png_watchdog"}
        """
        # PNG 回退与策略共享检测流，状态同步推进（随时可接管）
        png_cmd = self.png.step(det, roll, pitch, yaw, vx, vy, vz)

        obs = self.fb.build(det, roll, pitch, yaw, vx, vy, vz, local_z, dt)

        if self.watchdog_count > 0:
            self.watchdog_count -= 1
            return png_cmd.vx, png_cmd.vy, png_cmd.vz, png_cmd.yaw_rate, "png_watchdog"

        try:
            with torch.no_grad():
                action, self.h = self.model(
                    torch.from_numpy(obs).float().unsqueeze(0), self.h)
            a = action.squeeze(0).numpy()
            if not np.all(np.isfinite(a)):
                raise ValueError(f"策略输出非有限值: {a}")

            cvx, cvy, cvz, yaw_rate = decode_action(
                a, self.fb.los_v, self.fb.los_z, **self._decode_kw)
            speed = math.sqrt(cvx * cvx + cvy * cvy + cvz * cvz)
            if not math.isfinite(speed) or speed > self._speed_limit:
                raise ValueError(f"速度指令越界: |v|={speed:.2f}")

            return cvx, cvy, cvz, yaw_rate, "policy"

        except Exception as e:  # noqa: BLE001 —— watchdog 必须兜住一切异常
            self.watchdog_count = WATCHDOG_LATCH
            self.watchdog_total += 1
            # 隐藏状态可能已被污染（NaN 会沿 GRU 永久传播），重置后再恢复策略
            self.h = torch.zeros(1, 1, self.meta["gru_hidden"])
            if self.logger is not None:
                self.logger.warn(
                    f"[watchdog] 策略异常({e})，回退 PNG {WATCHDOG_LATCH} 帧 "
                    f"(累计 {self.watchdog_total} 次)")
            return png_cmd.vx, png_cmd.vy, png_cmd.vz, png_cmd.yaw_rate, "png_watchdog"
