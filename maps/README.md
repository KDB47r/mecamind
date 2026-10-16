# maps 目录

本目录存放第二节课（10.3.2）及以后产生的建图资产，统一使用项目内相对路径：

- `<名称>.pgm` + `<名称>.yaml`：nav2_map_server 格式的地图对。
- `<名称>_route_report.json`：半自动建图的路线执行报告。
- `<名称>_checkpoint_XX.*`：建图过程中的检查点快照（可选）。
- `*_manifest.json`：`mecamind_map_asset_manager` 生成的地图资产清单。

生成的地图默认不纳入版本控制（见根目录 `.gitignore`）。若要保留某张验收合格的地图供后续课程使用，用 `git add -f maps/xxx.pgm maps/xxx.yaml` 强制提交。
