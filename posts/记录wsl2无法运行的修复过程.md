---
title: WSL 2 无法启动与更新失败：一次修复记录
author: Daliang
published: 2025-08-10
updated: 2026-10-04
description: 记录 WSL 命令无响应、更新报错 1603 的现象、当时的处理经过，以及虚拟硬盘备份与恢复的注意事项。
toc: true
toc-depth: 4
toc-title: Contents
tags:
  - WSL
  - Ubuntu
category: "记录"
---

我遇到过一次 WSL 2 无法正常启动、更新也失败的故障。最后 Ubuntu 恢复了启动，但当时没有完整记录卸载、重新注册或恢复数据的过程，因此只能确认“环境恢复可用”，不能据此断言某一步就是根本修复原因。

## 故障现象

当时在 PowerShell 中执行 `wsl`、`wsl -l -v` 和 `wsl --shutdown` 都没有正常返回。尝试 `wsl --update` 时，安装程序报告旧版本无法移除：

```text
正在更新适用于 Linux 的 Windows 子系统: 2.5.10。
The older version of Windows Subsystem for Linux cannot be removed.
Contact your technical support group.
更新失败(退出代码: 1603)。
错误代码: Wsl/UpdatePackage/ERROR_INSTALL_FAILURE
```

这里的错误码能确认更新安装失败，但不足以单独定位原因。安装器提示的日志文件应保留，用于后续排查。

## 先区分 WSL 程序、发行版和数据

WSL 运行组件、Ubuntu 发行版应用和发行版里的文件是不同层次的问题。处理启动故障之前，需要先确认数据可以恢复。

此前的笔记曾写“找到 `ext4.vhdx`，压缩为 tar 后即可卸载”。这个表述不准确：**把 VHDX 文件打包成 tar，并不等于 `wsl --export` 导出的发行版 tar。** 两者不能混用恢复命令。

如果 WSL 命令仍可用，可以用官方导出命令生成备份。下面是备份方式示例，并非这次故障中已经成功执行的记录；`D:\WSL-Backup` 需要提前创建。

```powershell
wsl --shutdown
wsl --export Ubuntu-24.04 D:\WSL-Backup\Ubuntu-24.04.tar
```

如果命令已无响应，不能继续把上述导出命令当成可执行的备份方案。应先保留安装日志并排查 WSL 运行组件；若需复制原始 `ext4.vhdx`，必须先确保它未被挂载或写入，不能复制正在运行的发行版硬盘并宣称得到一致备份。备份的恢复方式需要用副本验证，不能在尚未确认可恢复时卸载发行版或执行 `wsl --unregister`；Microsoft 明确说明 unregister 会永久删除该发行版的数据、设置和软件。

官方分别提供 tar 导入和 VHDX 导入方式。恢复时应使用副本，并核对发行版名与存储位置，具体参数见 [Microsoft 的导出与导入说明](https://learn.microsoft.com/en-us/windows/wsl/basic-commands#export-a-distribution)。

## 当时的处理经过

当时参考了[褐瞳さん的 WSL 修复记录](https://www.hetong-re4per.com/posts/fixing-wsl-startup-issues)，尝试保留虚拟硬盘、卸载 Ubuntu 应用，并切换 Windows 的 WSL 功能后重启。

但笔记中没有记录可验证的备份、发行版重新注册及原数据恢复过程。这些操作可能涉及数据损失，因此不再把它们列为供读者照做的修复步骤。现有证据只能证明后来能进入 Ubuntu，不能证明这条操作链完整可靠，也不能确定哪一步解决了故障。

## 恢复后的验证

当时留下的日志显示，检查时的环境为：

| 项目 | 日志中的版本或状态 |
| --- | --- |
| Windows | 10.0.26100.4652 |
| WSL | 2.5.7.0 |
| Linux 内核 | 6.6.87.1-microsoft-standard-WSL2 |
| Ubuntu | 24.04.2 LTS |
| Ubuntu-24.04 | WSL 2，启动前状态为 Stopped |
| docker-desktop | Installing，不能据此确认 Docker 已恢复 |

用于验证的命令：

```powershell
wsl --version
wsl -l -v
wsl -d Ubuntu-24.04
```

当时已经能够进入 Ubuntu 终端。注意，更新失败时尝试安装的是 **2.5.10**，恢复后显示的是 **2.5.7.0**；这说明“能启动”不等于“已成功升级到目标版本”。

## 这次排障留下的经验

- 保留报错、安装日志和修复前后的版本，区分启动恢复与更新成功。
- 备份不仅是“文件复制完成”，还要明确备份格式及恢复路径。
- 进入 shell 后，还应检查项目文件、软件环境和挂载是否完整；这些检查未在原日志中记录。
- 记录中没有 Docker Desktop 恢复的验证结果，也没有确定更新安装失败根因的依据。

## 整理与核查说明

**核查状态：待复现（2026-10-04）。** 保留为历史故障记录；未验证完整备份恢复链条，已撤下可照做的卸载重装步骤。

2026-10-04 整理时更正了 VHDX 与 tar 备份格式的说明，并区分了历史操作和可供读者参考的备份示例。未重新执行卸载、恢复或重装操作，也没有补充原日志中未记录的数据恢复结论。

## 参考资料

- [Microsoft：WSL 基本命令](https://learn.microsoft.com/en-us/windows/wsl/basic-commands)
- [记录一下修复 WSL 无法启动的过程](https://www.hetong-re4per.com/posts/fixing-wsl-startup-issues)
