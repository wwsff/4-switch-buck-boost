# H750桌面电源固件发布包

Copyright (c) 2026 SZPUW — 原创应用代码GPL-3.0-only，第三方源码保留原授权。

配套版本：H750-NTC-CAN-20261008。工程入口：MDK-ARM/STM_POWER_LCD.uvprojx，ARM Compiler 5。应用源码和CubeMX配置均保留；输出目录缺省为空，首次使用应Rebuild，不要把旧构建输出当作新镜像。

当前配套镜像、AXF、构建记录和校验值只保留在Bench/20261008-ntc-can/。历史Release镜像、Git历史、SDK网页文档、预编译DSP库及宿主测试输出已从分发包去除；源码和测试保留，不改变固件行为。工程文件中引用的源码与包含目录已检查存在。

完整操作与已验证范围见整个开源目录README_开源使用说明.md。当前CH1、PROBE实验范围、未装IL快速过流和未启用新增过温停机的限制仍然适用；清理不等于这些项目已经验收。

宿主C测试脚本仍按开发机GCC路径编写，换电脑需核对工具路径。许可证见LICENSE/COPYRIGHT.md；第三方来源见THIRD_PARTY_NOTICES.md。

生成CubeMX代码后，从解压后外层目录运行tools/fix_keil_project.py，再运行check_ioc.py、check_ioc_intent.py并Rebuild。HSE配套8 MHz，SYSCLK225 MHz；不要套用旧25 MHz配置。8×16字体来源见第三方清单，授权范围尚待核实。
