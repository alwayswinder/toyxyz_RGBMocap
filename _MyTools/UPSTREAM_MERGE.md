# RGB MoCap 上游合并说明

本地扩展基于 `a7fde78`。`export/` 保存 Biped 手腕和手指校准、BVH 转 Max 的 Python 实现及 MAXScript。`correction/` 保存 AutoGenerate 使用的根位移平滑。`tests/` 保存导出回归测试。

## 原仓库修改点

| 文件 | 合并时保留的接入 |
| --- | --- |
| `mocap/biped_bvh.py` | 在 Character Studio 骨名与 ZXY 导出路径中调用手腕和手指校准。通用 BVH 继续走原路径。 |
| `mocap/source_export_dialog.py`、`mocap/gui.py`、`mocap/workflow_jobs.py` | SMPL 导出增加 `.max` 格式，保留 T-pose、移到原点、对称参考姿势、逐 take 导出和取消清理。 |
| `mocap/worker_progress.py`、`mocap/source_export_worker.py`、`mocap/export_copy.py` | `save_max` 复用 BVH worker，先生成 Biped BVH，再转换暂存文件。发布前检查源动作版本和场景位置，并保留原有覆盖策略。 |
| `mocap/worker.py`、`mocap/correction_settings.py`、`mocap/correction.py` | ExMR 的 `prepare` 在 `RGB_MOCAP_SMOOTH_ROOT_XZ=1` 时记录修正选项，接触求解前平滑水平骨盆补偿。普通修正默认关闭。 |
| `tools/bvh_to_max.py`、`pyproject.toml`、`tools/build_release.py` | 兼容 AutoGenerate 现有命令路径，发布包携带 `_MyTools` 和 MAXScript。 |
| `.gitignore`、`tools/check_source.py`、`SOURCE_MANIFEST.json`、`THIRD_PARTY_NOTICES.md`、`docs/USER_GUIDE.md` | 忽略本地导出，审查发布源文件，更新清单、授权和导出说明。 |

遇到冲突时，先采用上游的任务调度、相机选项和导出发布逻辑，再接回表中选项和调用。检查桌面菜单、worker 路由、安装包文件和自动化兼容入口。

## BVH 转 Max

从源码目录运行。默认输入为米制 Y-up 的 Humanoid52 BVH。对已校准的 FlowHMR 或 RGB MoCap Biped BVH 使用 `--profile biped`。需要本机已激活的 3ds Max。自定义安装位置可通过 `RGB_MOCAP_3DSMAX_BATCH` 指定。

```powershell
.\runtime\python.exe -I -X utf8 -B tools\bvh_to_max.py input.bvh output.max
.\runtime\python.exe -X utf8 -B -m unittest discover -s _MyTools/tests
```
