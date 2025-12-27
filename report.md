一、项目概述
1.1 作业背景

本次C++课程大作业要求我们深入理解并扩展一个大型C++项目——Google DeepMind的MuJoCo MPC。MuJoCo是一个高性能的物理仿真引擎，而MPC则是其配套的模型预测控制框架。作业的核心挑战在于，如何将一个典型的物理仿真应用（如车辆控制）与一个直观的用户界面（如汽车仪表盘）结合起来，从而展示仿真数据与图形渲染的协同工作。
1.2 实现目标

本项目的主要目标是：在MuJoCo MPC的3D仿真环境中，创建一个包含车身和车轮的简单车辆模型，并实现一个叠加在3D视图上的2D汽车仪表盘。该仪表盘需要能够实时显示从仿真中提取的关键驾驶数据，包括车辆速度、发动机转速、油量和水温等。
1.3 开发环境

    操作系统: Ubuntu 24.04 LTS
    编译器: gcc 13.3.0
    构建工具: CMake 3.28.1
    核心依赖: MuJoCo, OpenGL, GLFW, Eigen3

二、技术方案
2.1 系统架构

本项目采用“数据-逻辑-渲染”分离的架构思想。

    仿真层: 由 car_model.xml 定义，并由MuJoCo引擎负责物理计算。
    数据层: 由 simple_car.c/h 模块实现，负责从仿真层 (mjData) 提取原始物理量并转换为仪表盘所需的逻辑数据。
    渲染层: (待完全集成) 负责将数据层的输出通过OpenGL绘制为2D UI元素。

2.2 数据流程

数据流始于MuJoCo的每帧仿真循环。mjData 结构体包含了所有物体的位置、速度、力等状态信息。DashboardDataExtractor (在 simple_car.c 中实现) 会访问 mjData，计算出速度等派生数据，并填充到自定义的 DashboardData 结构体中。最终，DashboardRenderer 将读取此结构体并在屏幕上绘制对应的UI。
+-----------------+     +---------------------------------------+     +---------------------+     +------------------+
|                 |     |                                       |     |                     |     |                  |
|     mjData      | --> | getDashboardData(...)                 | --> |   DashboardData     | --> |   render(...)    |
|                 |     | (在 simple_car.c 中实现)              |     | (包含 speed, rpm...) |     | (绘制UI到屏幕)   |
+-----------------+     +---------------------------------------+     +---------------------+     +------------------+
        ^                             |                                      |
        |                             |                                      |
        |                             v                                      |
        +---------------------- (调用) ----------------------------> (调用)
2.3 渲染方案

本项目计划采用 正交投影 (Orthographic Projection) 的方式在3D视图的前景层绘制2D UI。在主渲染循环的末尾，会切换OpenGL的投影矩阵，从用于3D场景的透视投影切换到用于2D UI的正交投影。然后，调用 DashboardRenderer 的各个绘制函数（如 renderSpeedometer, renderTachometer）来完成仪表盘的绘制。
三、实现细节
3.1 场景创建

车辆场景在 mjpc/tasks/simple_car/car_model.xml 中定义。该文件基于作业文档的示例，创建了一个带有 freejoint 的车身（body name="car"）和两个通过 zaxis 约束的车轮（left wheel, right wheel）。同时，定义了两个肌腱（tendon）和马达（motor）来控制车辆的前进和转向。
图片在imagesimage1.png

3.2 数据获取

数据获取的核心在 simple_car.c 文件中。getDashboardData 函数是主要入口，它通过 d->qvel[0] 和 d->qvel[1] 获取车辆在世界坐标系X、Y轴上的线速度，计算合速度后，再换算为km/h，并按比例生成模拟的RPM值。

// simple_car.c - 核心数据提取逻辑 (片段)
void getDashboardData(const mjModel* m, const mjData* d, DashboardData* out) {
    // 假设车身是第0个Body，获取其X,Y方向的速度
    double vx = d->qvel[0];
    double vy = d->qvel[1];
    double speed_ms = sqrt(vx*vx + vy*vy); // m/s
    out->speed = speed_ms * 3.6; // Convert to km/h

    // 模拟转速: 速度为20m/s (72km/h)时，转速为4000 RPM
    out->rpm = fmin(8000.0, out->speed * 111.11);

    // 模拟油量和温度
    out->fuel = 100.0 - d->time * 0.1; // 随时间缓慢减少
    out->temp = 80.0 + out->speed * 0.1; // 随速度升高
}
此模块已通过在 app.cc 中添加 printf 语句进行初步验证，确认能正确输出随仿真时间变化的数值。

3.3 仪表盘渲染

3.3.1 速度表

速度表将是一个圆形表盘，内部包含一个可旋转的指针。渲染时，先绘制一个静态的表盘背景（可通过纯色或简单纹理实现），然后根据 DashboardData.speed 的值计算出指针的旋转角度，并使用 glRotatef 进行变换后绘制指针。

核心代码

   DrawGauge(speed_dashboard_pos, speed_ratio, speed_ticks,
            static_cast<float>(speed_kmh), "km/h",
            0.0f, 0.0f, 0.0f);

   const float max_speed_kmh = 10.0f;
   float speed_ratio = static_cast<float>(speed_kmh) / max_speed_kmh;
   if (speed_ratio > 1.0f) speed_ratio = 1.0f;
   if (speed_ratio < 0.0f) speed_ratio = 0.0f;

3.3.2 转速表

转速表的实现逻辑与速度表类似，但其警告区域（通常为红色）需要在RPM超过阈值（如6000）时被高亮。这可以通过在表盘背景之上，根据当前RPM值动态绘制一个扇形区域来实现。

图片在imagesimage2.png

核心代码

   DrawGauge(rpm_dashboard_pos, rpm_ratio, rpm_ticks,
            rpm_value, "RPM",
            0.0f, 1.0f, 0.0f);

   const float max_rpm = 8000.0f;
   float rpm_ratio = static_cast<float>(speed_m / max_speed_ref);
   if (rpm_ratio > 1.0f) rpm_ratio = 1.0f;
   if (rpm_ratio < 0.0f) rpm_ratio = 0.0f;
   float rpm_value = rpm_ratio * max_rpm;

  double angle_x = 90.0 * 3.14159 / 180.0;
  double cos_x = std::cos(angle_x);
  double sin_x = std::sin(angle_x);
  double mat_x[9] = {1, 0, 0, 0, cos_x, -sin_x, 0, sin_x, cos_x};

  double angle_z = -90.0 * 3.14159 / 180.0;
  double cos_z = std::cos(angle_z);
  double sin_z = std::sin(angle_z);
  double mat_z[9] = {cos_z, -sin_z, 0, sin_z, cos_z, 0, 0, 0, 1};

  double dashboard_rot_mat[9];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      dashboard_rot_mat[i * 3 + j] = 0;
      for (int k = 0; k < 3; k++) {
        dashboard_rot_mat[i * 3 + j] += mat_z[i * 3 + k] * mat_x[k * 3 + j];
      }
    }
  }

3.4 进阶功能（如果有）

计划实现的进阶功能包括：

    平滑动画：对指针的旋转使用线性插值（Lerp），使其运动更加平滑自然。
    警告提示：当转速过高或油量过低时，闪烁或高亮对应的UI元素。

四、遇到的问题和解决方案
问题1: 运行时提示 ERROR: Unknown command line flag 'mjcf'

    现象: 运行 ./bin/mjpc --mjcf=... 时，程序报错。
    原因: 命令行参数格式错误，正确的格式必须包含等号 =。
    解决: 将命令修正为 ./bin/mjpc --mjcf=../mjpc/tasks/simple_car/car_model.xml。

问题2: 数据获取依赖硬编码的Body ID

    现象: 如果在 car_model.xml 中修改了Body的顺序，getDashboardData 函数将无法正确获取车身速度。
    原因: 代码中直接使用了 d->qvel[0]，假设了车身是模型中的第一个Body。
    解决: 后续将使用 mj_name2id(m, mjOBJ_BODY, "car") 函数通过名称查找Body ID，再根据ID计算出对应的 qvel 索引，使代码更具鲁棒性。

五、测试与结果
5.1 功能测试

通过在 app.cc 的主循环中打印 DashboardData 的内容，验证了速度、转速等数据能随仿真进行而实时更新，功能符合预期。
5.2 性能测试

由于渲染部分尚未完全集成，目前无法进行完整的性能测试。预计在实现正交投影渲染后，通过内置的 mj_activateProfiler() 可以对帧率和资源占用进行评估。

5.3 效果展示
图片在imagesimage2.png
视频在videos

六、总结与展望
6.1 学习收获

通过本次大作业，我深入学习了大型C++项目的组织结构、CMake构建系统、以及MuJoCo物理引擎的核心API。更重要的是，我掌握了如何将底层数据（物理仿真）与上层表现（图形渲染）进行有效解耦和集成。
6.2 不足之处

目前最大的不足是仪表盘的OpenGL渲染部分尚未完全集成到主程序中，导致无法直观地看到最终效果。此外，数据获取部分的健壮性有待提高。
6.3 未来改进方向

    完成仪表盘UI的OpenGL渲染，并将其回调函数集成到 mjpc 的主循环中。
    优化 simple_car.c 中的数据获取逻辑，使用动态ID查找，解除硬编码依赖。
    添加真实的车辆控制逻辑（如键盘控制或简单的MPC控制器），使仿真更具交互性。

七、参考资料

    MuJoCo 官方文档: https://mujoco.readthedocs.io/
    LearnOpenGL CN: https://learnopengl-cn.github.io/
    《C++ Primer》(第5版)
    课程大作业文档《大作业.md》

学号：232011081 姓名：李佩靖 完成日期：2025年12月27日













