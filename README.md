# TcCppLib

![example workflow](https://github.com/Link2Link/TcCppLib/actions/workflows/CI.yml/badge.svg)
[![codecov](https://codecov.io/gh/Link2Link/TcCppLib/branch/main/graph/badge.svg?token=HLMZ1YV51X)](https://codecov.io/gh/Link2Link/TcCppLib)
[![CodeFactor](https://www.codefactor.io/repository/github/link2link/tccpplib/badge)](https://www.codefactor.io/repository/github/link2link/tccpplib)

本项目旨在为Twincat3 C++ 实时环境提供更丰富的数学、机器人、控制库。
### 版本需求
Twincat版本大于 3.1.4024.35

### 使用指南
为在倍福环境中使用本项目，需要在C:\TwinCAT\3.1\sdk\Include\TcMath\math_config.h文件头部添加宏定义
```angular2html
#define TCMATH_BLOCK_STANDARDLIB 0
```

### 可靠度评级

| 等级  |      指标       |
|:---:|:-------------:|
|  A  |   完善测试且广泛使用   |
|  B  | 完善测试或在实际中经过验证 |
|  C  | 一般测试，未在项目中使用  |
|  D  |     未经测试      | 


### 移植的库

- [[A] Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) : 裁剪了半浮点运算相关运算
- [[B] MR](https://github.com/Le0nX/ModernRoboticsCpp.git) : MR的cpp版本
- [[C] PPX](https://github.com/Xtinc/matrix) : 裁剪了statistics部分
- [D] CHEN : 开发中



