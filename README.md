# TcCppLib

![example workflow](https://github.com/Link2Link/TcCppLib/actions/workflows/CI.yml/badge.svg)
[![codecov](https://codecov.io/gh/Link2Link/TcCppLib/branch/main/graph/badge.svg?token=HLMZ1YV51X)](https://codecov.io/gh/Link2Link/TcCppLib)
[![CodeFactor](https://www.codefactor.io/repository/github/link2link/tccpplib/badge)](https://www.codefactor.io/repository/github/link2link/tccpplib)

本项目旨在为Twincat3 C++ 实时环境提供更丰富的数学、机器人库。
### 版本需求
Twincat版本大于 3.1.4024.35

### 使用指南
为在倍福环境中使用本项目，需要在C:\TwinCAT\3.1\sdk\Include\TcMath\math_config.h文件头部添加宏定义
```angular2html
#define TCMATH_BLOCK_STANDARDLIB 0
```

### 移植的库

- [PPX](https://github.com/Xtinc/matrix) : 裁剪了statistics部分
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) : 裁剪了半浮点运算相关运算
- [MR](https://github.com/Le0nX/ModernRoboticsCpp.git) : MR的cpp版本

