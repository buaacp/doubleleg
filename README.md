# Doubleleg 模块化机械腿 CAD

这是当前的参数化机械腿发布包，包含生成代码、设计参数、完整装配 STEP、逐件 STEP、预览图、BOM、连接矩阵与装配规范。

设计采用双侧平板杆件、模块化双侧支撑关节和 V5 足爪。髋—膝轴距为 158.1708 mm，膝—踝轴距为 163.5926 mm。关节电机目前仍是不绑定具体型号的 X430 级封装参考，不表示其满足 12 N·m 峰值需求。

## 快速开始

```bash
python -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python build.py
```

生成结果位于 `output/`。更详细的设计说明见 `README_CN.md`，精确装配顺序和紧固件规范见 `全腿装配与连接规范_R2.md`。

## 主要输出

- `output/leg_assembled.step`：全腿装配
- `output/joint_module_assembled.step`：关节模块装配
- `output/joint_module_exploded.step`：关节爆炸图
- `output/parts/`：可制造零件 STEP
- `engineering_BOM.csv`：工程 BOM
- `connection_matrix.csv`：零件连接表
- `output/validation.json`：生成与几何校验结果

`vendor/v5_foot_parts/` 保存构建全腿所需的 V5 足爪 STEP，因此仓库可独立重建。
