# 夹爪笔记的最小空载验证

核查日期：2026-10-05。来源是用户提供的 `parallel_gripper_tactile` 仓库，固定提交为 `b5b10deaa1b15f05f71aec59cfeb2d813d251656`，资产目录为 `assets/grippers/dm_gripper/`。本例不运行来源仓库的控制程序，也不重新导出 CAD。

## 运行

需要 Python 3.13 和上述提交的资产目录。将三份 XML 与同目录 `assets/` 保持完整。依赖版本见 `requirements.txt`，建议安装到单独的虚拟环境。

在博客内容仓库根目录执行：

```text
python -m pip install -r examples/gripper-verification/requirements.txt
python examples/gripper-verification/verify.py --asset-dir PATH/TO/dm_gripper --output-dir PATH/TO/results --source-commit b5b10deaa1b15f05f71aec59cfeb2d813d251656
```

`--source-commit` 是来源记录，脚本不会替你核对 Git 工作区；运行前应确认资产确实来自该提交。报告同时记录 XML 与全部 27 个 STL 的 SHA-256，可与保存的报告核对。

## 验证范围

1. 分别编译 `parallel_gripper.xml`、`parallel_gripper_prepared.xml`、`parallel_gripper_height_sphere_collision.xml`，核对单执行器和四条闭环 connect。
2. 在内存中移除 `base_freejoint`，将基座固定在世界；保留回差关节、惯量、摩擦、碰撞和求解参数，不改写任何源文件。
3. 通过外部 PD 产生力矩，限幅沿用 XML 的 ±4 N·m；目标角在 0～1.2 rad 内按 10 秒周期平滑变化，运行两个周期。PD 参数是这个测试的选择，不是已标定的硬件控制器。
4. 记录四对闭环 site 的世界坐标距离、左右滑台位移、指尖 site 法向的平行角、驱动跟踪误差、接触数量和仿真警告。

空载检查阈值在脚本中预先指定：最大闭环误差小于 0.1 mm、法向平行角小于 0.1°、每侧滑台移动超过 5 mm、没有仿真警告。它们只是本例检查标准，不代表硬件精度或接触性能指标。每个时步还检查状态、速度和加速度均为有限值。

输出包含完整 CSV、JSON、运行曲线。报告中的三个模型空载曲线相同，不说明它们的接触响应相同；本次各模型接触数量均为零。

## Windows 资源读取

本机 MuJoCo 3.4.0 与 3.10.0 直接从磁盘加载时，都在 `part_1__配置.stl` 处报文件打开错误，文件本身存在。脚本用 Python 读取原始字节，通过 MuJoCo 虚拟文件系统（VFS）提供 ASCII 文件别名；显式保留原 mesh 名称，几何字节与尺度不变。这验证了资产通过该加载路径可编译，不能写成原始文件在所有平台都可直接打开。

App/导出器的原始版本仍未记录；这不是 CAD 导出流程的复现。没有测试物体接触、抓取、触觉标定、实机力控或硬件精度。来源资源仍在原仓库，未复制网格到博客。
