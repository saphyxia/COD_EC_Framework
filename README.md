**COD EC HAL Template**

# HAL-Template

## 简介

* 开发工具：keil V5.38a，VsCode
* 软件环境：Window11
* 硬件环境：大疆RoboMaster开发板C型(STM32F407IGHX)
* 编译工具：Arm Compiler V5.06u7，C/C++编译

## 目录结构

```
HAL-Template
  ├───Application/API
  ├───Application/Tasks
  ├───BSP
  ├───Components/Algorithm
  ├───Components/Controller
  ├───Components/Device
  ├───Core
  ├───Docs
  ├───Drivers
  ├───MDK-ARM
  ├───Middlewares
  ├───Third_Party
  └───USB_DEVICE
```

## 模块功能说明

待完善

## 贡献

* 完善项目过程中，请尽量遵循以下设计原则和规范：
  * `API` 应用接口层对应应用接口，是一类功能的抽象，请不要在该层相关文件中定义实体变量；该层调用各组件层以实现功能；
  * `Bsp` 板级支持包面向底层组件，是唯一允许直接出现STM32HAL库函数的代码层；
  * 请不要跨层调用；
  * 请注意`代码规范`，建议参考[Google C++风格指南](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/contents/#).

* 欢迎提交Issues和Pull Requests帮助我们改进.