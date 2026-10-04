---
title: 使用 Mizuki 搭建 Astro 博客并部署到 GitHub Pages
published: 2025-08-22
updated: 2026-10-04
description: 记录 Mizuki 博客的本地配置与 GitHub Pages 部署，区分用户站点和项目站点，并整理常见配置错误。
tags: [Blog, Mizuki, Astro]
category: "记录"
author: Daliang
---
这篇文章记录我在 2025 年使用 Mizuki 搭建 Astro 博客并部署到 GitHub Pages 的过程。主题配置文件和 Actions 版本会变化，下面的历史配置应结合所用主题版本核对。

**阅读路线：**准备本地环境 → 配置文章与站点 → 区分部署网址 → 使用 GitHub Actions 构建发布。

需要准备 Git、GitHub 账号，以及符合项目要求的 Node.js 和包管理器。版本要求以所用主题的 `package.json` 和官方说明为准。

## 基于Mizuki主题模板创建起始项目

原先使用的主题是 [Mizuki](https://github.com/matsuzaka-yuki/Mizuki)，配置细节参考[主题文档](https://docs.mizuki.mysqil.com/)。

从模板创建自己的仓库，或 Fork 后克隆自己的仓库。只有用户/组织主页需要命名为 `<用户名>.github.io`；项目站点可以使用其他仓库名，网址通常多一层仓库路径。

```powershell
git clone  https://github.com/你的用户名/你的仓库名.git my-blog
cd my-blog
```

### 本地配置与预览

优先使用项目声明的包管理器，避免混用 npm 和 pnpm 生成两份锁文件。以 pnpm 项目为例，在 Windows PowerShell 中可以使用：

```powershell
pnpm.cmd install
pnpm.cmd dev
```

随后完成三类配置：

1. **站点信息：**标题、作者、语言和公开网址。旧版 Mizuki 可能集中在 `src/config.ts`，其他版本可能拆分到 `src/config/`，以实际项目为准。
2. **内容：**文章通常位于 `src/content/posts/`；当前博客已采用独立内容仓库，需要通过对应同步流程接入主题。
3. **静态资源：**核对头像、文章图片和封面路径，并在本地预览中逐项检查。

不要把含 `posts/` 的内容仓库直接当作 Astro 程序运行；实际构建入口位于主题/站点项目。

## 部署Astro站点到GitHub Pages

### 区分用户站点与项目站点

Astro 的选项名是 `site`，不是 `size`。`site` 设置公开网址，`base` 用于指定子路径。用户站点部署在域名根目录时，一般不需要额外的 `base`：

```javascript
import { defineConfig } from 'astro/config';

export default defineConfig({
  site: 'https://你的用户名.github.io',
});
```

如果部署的是名为 `my-blog` 的项目站点，配置示例为：

```javascript
export default defineConfig({
  site: 'https://你的用户名.github.io',
  base: '/my-blog',
});
```

以上仅展示相关选项，应合并到现有配置中，保留主题的 integrations 等设置。自定义域名或主题自动生成配置时，应以最终访问地址和主题入口为准。

### 配置 GitHub Actions

下面按 2026-10-04 查阅的 [Astro 官方部署示例](https://docs.astro.build/en/guides/deploy/github/)更新 Actions 版本。旧文的 `withastro/action@v3` 和默认 Node 20 注释不应继续作为新建站点的当前配置；旧版本能否继续构建仍需结合项目验证。

示例适用于含 Astro 程序和锁文件的站点仓库。当前独立内容仓库还需要站点自己的内容同步步骤，以及内容更新后触发站点构建的机制；直接复制该工作流不会自动同步另一仓库的文章。Node 与包管理器应匹配主题要求，并提交相应锁文件。

1. 在站点项目的 `.github/workflows/` 创建 `deploy.yml`：

   ```yaml
   name: Deploy to GitHub Pages

   on:
     # 每次推送到 `main` 分支时触发这个“工作流程”
     # 如果你使用了别的分支名，请按需将 `main` 替换成你的分支名
     push:
       branches: [ main ]
     # 允许你在 GitHub 上的 Actions 标签中手动触发此“工作流程”
     workflow_dispatch:

   # 允许 job 克隆 repo 并创建一个 page deployment
   permissions:
     contents: read
     pages: write
     id-token: write

   jobs:
     build:
       runs-on: ubuntu-latest
       steps:
         - name: Checkout your repository using git
           uses: actions/checkout@v7
         - name: Install, build, and upload your site
           uses: withastro/action@v6
           # with:
             # path: . # 存储库中 Astro 项目的根位置。（可选）
             # node-version: 24 # 此版本 action 默认使用 24；若主题有其他要求，显式设置。
             # package-manager: pnpm@指定版本 # 按项目锁定的版本填写；默认会检测锁文件。

     deploy:
       needs: build
       runs-on: ubuntu-latest
       environment:
         name: github-pages
         url: ${{ steps.deployment.outputs.page_url }}
       steps:
         - name: Deploy to GitHub Pages
           id: deployment
           uses: actions/deploy-pages@v5
   ```
2. 在 GitHub 上，跳转到存储库的 **Settings** 选项卡并找到设置的 **Pages** 部分
3. 选择 **GitHub Actions** 作为你网站的 **Source**，然后按 **Save**。
4. 提交（commit）这个新的“工作流程文件”（workflow file）并将其推送到 GitHub

如果已经克隆自己的仓库，远程地址通常已经存在，可以先检查，再提交并推送本次改动：

```powershell
git remote -v
git add .github/workflows/deploy.yml
git commit -m "配置 GitHub Pages 部署"
git push
```

工作流中的分支名应与实际发布分支一致。若改动了站点配置，也需要把对应文件纳入提交。构建成功后，继续检查首页、文章页、图片、公式、搜索和子路径链接，而不只看 Actions 是否变绿。

## 常见问题

- **图片或样式 404：**核对 `base` 是否匹配公开网址，以及资源路径是否支持子路径部署。
- **内容更新未出现：**检查提交是否进入发布分支，独立内容仓库是否成功同步，构建是否重新执行。
- **依赖安装失败：**核对 Node.js、包管理器版本和锁文件，不直接删除锁文件试错。
- **主题升级后配置失效：**对照所用版本的配置说明重新映射设置。

## 整理与核查说明

**核查状态：部分验证（2026-10-04）。** 已核对官方部署方法；本轮迁移的实际部署结果另行记录，不把旧工作流示例当作当前模板。

2026-10-04 整理时修正了链接、命令和部署路径说明。文中的配置以当时的 Mizuki 使用经历为背景；主题与 Actions 版本会变化，实际部署仍须结合所用版本和仓库类型核对。

## 参考资料


- [Astro：部署到 GitHub Pages](https://docs.astro.build/zh-cn/guides/deploy/github/)
- [图床设置](https://www.cnblogs.com/liangfengshuang/p/18474356)
- [原笔记参考文章](https://www.cnblogs.com/misakivv/p/18593896)
