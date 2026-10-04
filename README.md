# 学习与实践笔记

正文唯一来源是 `posts/`，配图在 `images/` 或文章同目录。“我”主要用于开头交代经历，正文直接说明方法与结果。知乎迁入稿保留来源、可见日期和核查范围，不自动修改知乎原文。

## 写作、外观和发布

- `posts/`：文章，`draft: true` 仅本地可见，生产构建过滤。
- `spec/about.md`：关于页。
- `site/shirones/config/`：个人外观和功能配置；直接修改这里，构建不会重置。
- `site/package.json` 和锁文件：Shirone npm 主题依赖，当前为 0.1.5；升级前先本地检查。
- `docs/`：核查范围、图片来源、迁移验收和写作模板。
- `site/shirones/content/`：自动生成的内容副本，不编辑、不提交。

需要 Node.js 24 LTS 和 pnpm 12.6.0。首次在 `site/` 安装依赖，再运行 `pnpm dev`；Obsidian 保存 `posts/`、`spec/`、`images/` 后会同步到预览。`pnpm check` 检查配置，`pnpm build` 构建公开内容并检查旧链接、图片、公式和订阅地址。

正式地址为 <https://langxin11.github.io/>。内容仓库推送后，发布仓库每小时检查一次更新；GitHub 排队可能延迟。需要立即发布时，在已提交且干净的 main 分支运行 `./scripts/Publish-Shirone.ps1`。构建或验收失败不会替换线上站点。

包含 4 篇草稿的旧预览仍可用 `./scripts/Start-ShironePreview.ps1`；生成产物有草稿提示，不能上传为正式网站。

详细步骤见 [正式发布与维护](docs/正式发布与维护.md)，文章验证范围见 [准确性审查](docs/文章准确性与时效性审查.md)。
