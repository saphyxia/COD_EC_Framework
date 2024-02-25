# COD_EC_Framework

## 开发环境

* 开发工具：Keil MDK-ARM V5.39, Visual Studio Code
* 软件环境：Window11
* 硬件环境：DJI RoboMaster开发板C型 (STM32F407IGHX)
* 编译工具：Arm Compiler v6.21

## 文件结构

```
COD_EC_Framework
  ├───Algorithm
  ├───Controller
  ├───Core
  ├───Drivers
  ├───MDK-ARM
  ├───Middlewares
  ├───Modules
  ├───Tasks
  └───Third_Party
```

## 模块功能说明

### IMU 惯性测量单元

* 参考[哈尔滨工程大学创梦之翼战队惯导姿态解算项目](https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example)。

* 详情见[IMU.md](./Docs/IMU.md)

## 贡献

* 完善项目过程中，请尽量遵循以下设计原则和规范：
  * 请不要跨层调用；
  * 请注意`代码规范`，建议参考[Google C++风格指南](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/contents/#).
  
* 欢迎提交Issues和Pull Requests帮助改进。