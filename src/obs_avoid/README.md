# obs_avoid

这个项目是复现一个用视觉分割模型来避障的项目。

## 项目规范

> 由于cpp不像python那样风格比较确定，所以这里列出一些该项目遵守的规则

[参考文档](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)

1. c++使用17标准，使用cmake构建，使用git管理代码。
2. 头文件使用hpp后缀，源文件使用cpp后缀。
3. 全局变量g_xxx命名
4. 静态变量使用s_xxx命名
5. 类名为CamelCase
6. 函数名使用CamelCase
7. 局部变量与函数参数使用aaBbCc命名，类成员变量使用m_AaBbCc命名
8. 其余风格不做要求，只要看得下去即可
