# 第三方内容说明

本项目原创内容的作者为SZPUW，年份2026。下列第三方内容继续采用原许可证，原文件与版权注释已保留。

| 内容 | 原包中的许可证位置 | 本次处理 |
|---|---|---|
| STM32G4/H7 HAL与ST设备支持 | Drivers/STM32G4xx_HAL_Driver/LICENSE.txt、Drivers/STM32H7xx_HAL_Driver/LICENSE.txt及CMSIS/Device/ST相应LICENSE.txt | 保留原文件；HAL文件引用Package_license，未随包提供时声明BSD-3-Clause适用；不覆盖其授权 |
| ARM CMSIS | Drivers/CMSIS/LICENSE.txt；DSP/ComputeLibrary下另有许可证 | 保留原Apache-2.0等文件和源码版权；不覆盖其授权 |
| FreeRTOS | Middlewares/Third_Party/FreeRTOS/Source/LICENSE | 保留Amazon版权与原MIT条款 |
| pyserial | PowerDesk requirements.txt声明pyserial==3.5 | 作为外部依赖安装，未把第三方安装包并入本发布包；其许可按上游包处理 |
| H750 8×16 ASCII字体 | User/Ui/Inc/font_8x16.h说明来自屏幕商家示例LCD_Demo_taobao/HARDWARE/LCD/FONT.H的asc2_1608 | 保留来源与数据。作者表示可以使用，但未提供商家明确再分发授权；不能认定其已由SZPUW按GPL授权，公开分发权限尚待核实 |

本次做的是保留许可证、登记来源与发布包清理，不是对全部第三方依赖做完法律兼容性审查。字体是明确尚未找到书面授权的一项；如无法确认，应在后续版本换为授权清晰的字库，再重新构建、测试与更新镜像。本次没有替换字体或改动固件行为。
