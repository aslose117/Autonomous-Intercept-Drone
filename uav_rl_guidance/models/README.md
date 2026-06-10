# 模型目录

放置训练产物（guidance_rl/export.py 导出）：

```bash
cd /home/verser/Python/guidance_rl
python -m guidance_rl.export --ckpt checkpoints/rl_policy.pt \
    --out /home/verser/ros2_ws/src/uav_rl_guidance/models/policy.pt
```

生成两个文件：
- `policy.pt`       — TorchScript 策略（GRU Actor，CPU 推理）
- `policy_meta.json` — 动作解码常数 + 特征版本（运行时校验）
