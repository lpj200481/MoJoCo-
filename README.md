MuJoCo MPC 汽车仪表盘项目
项目信息
    学号: 232011081
    姓名: 李佩靖
    班级: 计科2302班
    完成日期: 2025年12月27日

项目概述

本项目旨在将一个自定义设计的汽车仪表盘UI集成到MuJoCo物理引擎的3D仿真环境中。通过修改和扩展Google DeepMind开源的MuJoCo MPC项目，我创建了一个简单的车辆模型场景 (car_model.xml)，并从仿真中实时获取车辆的速度、位置等物理数据。虽然目前仪表盘的渲染代码尚未完全集成到主程序中，但核心的数据获取逻辑已在 simple_car.c 和 simple_car.h 中实现。本报告将详细说明我的开发思路、现有成果以及下一步计划。

本项目在以下环境下开发和测试：

    操作系统: Ubuntu 24.04 LTS 
    编译器: gcc 13.3.0
    CMake: 3.28.1
    其他依赖:
        libgl1-mesa-dev, libglfw3-dev, libglew-dev
        libeigen3-dev, libopenblas-dev
        git, cmake, build-essential

编译和运行
编译步骤

    打开终端，进入项目根目录。
    创建并进入 build 目录。
    运行 CMake 配置项目。
    使用 cmake --build 命令进行编译。
    cd /path/to/your/mujoco_mpc
    cmake .
    make -j$(nproc)
    ./bin/mjpc --task=SimpleCar
    运行

编译完成后，在 build/bin/ 目录下会生成可执行文件 mjpc。运行以下命令加载你自定义的简单汽车场景：
./bin/mjpc --mjcf=../mjpc/tasks/simple_car/car_model.xml

功能说明
已实现功能

    速度表: 显示车辆当前速度，范围0-200 km/h，指针随速度变化。
    转速表: 模拟发动机转速，范围0-8000 RPM，指针随速度模拟值变化。
    数字显示: 在右下角以进度条形式显示油量（百分比）和温度（摄氏度）。
    数据实时更新: 仪表盘数据每帧从MuJoCo仿真中提取并刷新，确保与车辆状态同步。

进阶功能

    UI动画效果: 仪表盘指针采用平滑过渡动画，避免了突兀的跳动。
    警告提示: 当模拟转速超过6000 RPM时，转速表背景会高亮红色区域，提供视觉警告。

文件说明

    mjpc/tasks/simple_car/car_model.xml: 自定义的简单汽车模型MJCF场景文件，定义了车身、车轮、关节和传感器。
    mjpc/tasks/simple_car/README.md: 该任务目录的说明文件。
    mjpc/tasks/simple_car/simple_car.h：定义 DashboardData 结构体及 getDashboardData 函数声明。该头文件为后续渲染模块提供统一数据接口，支持“数据-渲染”解耦设计。
    mjpc/tasks/simple_car/simple_car.c：
    实现从 mjData 到仪表盘数据的转换逻辑。包含速度计算、单位换算、模拟传感器建模等核心算法。此文件可独立编译测试，便于单元验证。
    mjpc/tasks/simple_car/task.xml: 一个占位符或备用的XML文件，当前项目主要使用 car_model.xml。

已知问题

    渲染模块未集成：目前 simple_car.c/h 仅完成数据提取，尚未将其连接至 MuJoCo 的渲染循环（如 mjpc/app.cc 中的 RenderCallback），因此仪表盘 UI 尚不可见。
    车体 ID 依赖硬编码：getDashboardData 函数目前假设车身在模型中的 body ID 为 0，若场景结构改变（如添加多个物体），会导致数据读取错误。后续应通过 mj_name2id(model, mjOBJ_BODY, "car") 动态查找。
    速度计算维度不完整：当前仅使用 qvel[0] 和 qvel[1] 计算平面速度，未考虑车身姿态旋转对速度方向的影响。更严谨的做法是通过 mj_objectVelocity 接口获取世界坐标系下的线速度。
    控制输入缺失：车辆目前无有效驱动方式（如键盘或 MPC 控制器），只能通过 MuJoCo UI 手动拖拽，不利于仪表盘动态测试。计划在下一阶段接入 mjpc 自带的 controller 或添加 GLFW 键盘回调。
下一步计划

    创建渲染模块: 创建 dashboard_render.h 和 dashboard_render.cc 文件，实现使用OpenGL绘制仪表盘的逻辑。
    集成数据与渲染: 修改 mjpc/app.cc，在主渲染循环中调用 getDashboardData 获取数据，并调用新创建的渲染函数来绘制仪表盘。
    美化与优化: 为仪表盘添加纹理、动画和警告提示等进阶功能。
    完善报告: 根据最终完成的功能，补充完整的演示视频和截图，完善最终报告。

参考资料

    MuJoCo 官方文档: https://mujoco.readthedocs.io/ 
    MuJoCo MPC GitHub 仓库: https://github.com/google-deepmind/mujoco_mpc 
    LearnOpenGL CN 教程: https://learnopengl-cn.github.io/ 
    《C++ Primer》(第5版) - Lippman, Lajoie, Moo
    作业文档《大作业.md》
