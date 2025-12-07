---
title: 记录wsl2无法运行的修复过程
author: Daliang
published: 2025-08-10
toc: true
toc-depth: 4
toc-title: Contents
tags:
  - wsl2
  - Ubuntu
category: "记录"
---
目前在终端中执行wsl命令无法运行，包括 `wsl -l -v`,`wsl-shutdown`等运行后没有反应
执行 `wsl -update`返回如下结果

```powershell
PS C:\Users\26583> wsl --update
正在检查更新。
正在更新适用于 Linux 的 Windows 子系统: 2.5.10。
The older version of Windows Subsystem for Linux cannot be removed.  Contact your technical support group.
更新失败(退出代码: 1603)。
日志文件: C:\Users\26583\AppData\Local\Temp\wsl-install-logs.txt
错误代码: Wsl/UpdatePackage/ERROR_INSTALL_FAILURE
```

目前决定卸载wsl 进行重装，参考 `褐瞳さん`的博客/[记录一下修复 WSL 无法启动的过程](https://www.hetong-re4per.com/posts/fixing-wsl-startup-issues)

1. 首先将已安装的ubuntu24.04 进行备份
   通过everything 搜索 `ext4.vhdx`找到虚拟硬盘的位置，压缩成tar文件放在同一文件夹即可（也可以移动到其他文件夹）
2. 卸载Ubuntu 24.04LTS 发行版
   在设置（`win` +`i`）的应用->安装的应用，找到你所安装的发行版，点击卸载
3. 关闭[适用于 Linux 的 Windows 子系统]
   在控制面板->程序和功能找到「启用或者关闭 Windows 功能」，找到[适用于 Linux 的 Windows 子系统]，取消勾选。按照提示重启电脑
4. 开启[适用于 Linux 的 Windows 子系统]
5. 在命令行查看

```powershell
PS C:\Users\26583> wsl --version
WSL 版本: 2.5.7.0
内核版本: 6.6.87.1-1
WSLg 版本: 1.0.66
MSRDC 版本: 1.2.6074
Direct3D 版本: 1.611.1-81528511
DXCore 版本: 10.0.26100.1-240331-1435.ge-release
Windows: 10.0.26100.4652


PS C:\Users\26583> wsl -l -v
  NAME              STATE           VERSION
* Ubuntu-24.04      Stopped         2
  docker-desktop    Installing      2


PS C:\Users\26583> wsl
Welcome to Ubuntu 24.04.2 LTS (GNU/Linux 6.6.87.1-microsoft-standard-WSL2 x86_64)

 * Documentation:  https://help.ubuntu.com
 * Management:     https://landscape.canonical.com
 * Support:        https://ubuntu.com/pro

 System information as of 2025年 08月 10日 星期日 20:14:39 CST

  System load:  0.0                 Processes:             101
  Usage of /:   0.6% of 1006.85GB   Users logged in:       0
  Memory usage: 3%                  IPv4 address for eth0: 172.27.193.30
  Swap usage:   0%

 * Strictly confined Kubernetes makes edge and IoT secure. Learn how MicroK8s
   just raised the bar for easy, resilient and secure K8s cluster deployment.

   https://ubuntu.com/engage/secure-kubernetes-at-the-edge

This message is shown once a day. To disable it please create the
/home/xiaodaliang/.hushlogin file.
xiaodaliang@LAPTOP-9F8FQIFM:/mnt/c/Users/26583$
```
