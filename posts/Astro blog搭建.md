---
title: 基于Mizuki的Astro模板个人博客搭建
published: 2025-08-22
tags: [Blog, Mizuki,Astro]
category: "记录
"
author: Daliang
---
# Astro博客搭建

主要借助Mizuki模板搭建个人博客网站，并且部署到Github pages，需要下载Node.js和Git工具和一个Github账号

## 基于Mizuki主题模板创建起始项目

模板地址：https://https://github.com/matsuzaka-yuki/Mizuki，官方的使用文档https://docs.mizuki.mysqil.com/press/file/，馋哭我了

Fork 原仓库，手动克隆仓库（仓库名必须为：`<你的用户名>.github.io`）

```powershell
git clone  https://github.com/你的用户名/yukina.git my-blog
cd my-blog
```

**本地配置博客**

1. 安装依赖

   ```
   npm install
   ```
2. 修改博客信息 `src/config.ts`(具体页面布局见文档)
3. 添加文章：在 `src/content/posts`新建markdown文件，Frontmatter需包含必填字段
4. 本地查看

   ```
   npm install g pnpm
   pnpm run
   pnpm dev
   ```

## 部署Astro站点到GitHub Pages

### 部署到 `github.io`网址

1. 创建**`.github/workflows/deploy.yml`**
2. 在 `astro.config.mjs`中配置文件设置 `size`和 `base`选项，其中 `base`可不提供

   ```
   import { defineConfig } from 'astro/config'

   export default defineConfig({
     site: 'https://langxin11.github.io',
     base: 'my-repo',
   })
   ```

### 配置GitHub Action

1. 在项目的 `.github/workflows`创建一个deploy.yml，粘贴一下yaml配置信息

   ```
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
           uses: actions/checkout@v4
         - name: Install, build, and upload your site
           uses: withastro/action@v3
           # with:
             # path: . # 存储库中 Astro 项目的根位置。（可选）
             # node-version: 20 # 用于构建站点的特定 Node.js 版本，默认为 20。（可选）
             # package-manager: pnpm@latest # 应使用哪个 Node.js 包管理器来安装依赖项和构建站点。会根据存储库中的 lockfile 自动检测。（可选）

     deploy:
       needs: build
       runs-on: ubuntu-latest
       environment:
         name: github-pages
         url: ${{ steps.deployment.outputs.page_url }}
       steps:
         - name: Deploy to GitHub Pages
           id: deployment
           uses: actions/deploy-pages@v4
   ```
2. 在 GitHub 上，跳转到存储库的 **Settings** 选项卡并找到设置的 **Pages** 部分
3. 选择 **GitHub Actions** 作为你网站的 **Source**，然后按 **Save**。
4. 提交（commit）这个新的“工作流程文件”（workflow file）并将其推送到 GitHub

```
git add .
git commit -m "初始提交"
git branch -M main
git remote add origin https://github.com/yourname/yourname.github.io.git
git push -u origin main
```

参考

+ https://docs.astro.build/zh-cn/guides/deploy/github/
+ gitee[图床设置](https://www.cnblogs.com/liangfengshuang/p/18474356)
+ https://www.cnblogs.com/misakivv/p/18593896
