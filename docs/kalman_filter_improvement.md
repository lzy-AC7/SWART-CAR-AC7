# 卡尔曼滤波测速和测距离改进方案

## 概述

用**卡尔曼滤波器**替代原有的简单互补滤波，实现编码器和加速度计的最优融合测速与测距离。

## 核心改进

### 1. 原方案的问题
```c
// 原方案：简单互补滤波（当ACC_ALPHA=0时完全不用加速度计）
v_x_car = ACC_ALPHA * v_x_imu*1000 + (1-ACC_ALPHA) * v_x_car;
```
- **固定权重**：权重ACC_ALPHA不会自适应调整
- **单次加速度积分漂移**：加速度积分累积误差，容易漂移
- **编码器延迟大**：编码器有0~5ms的固有延迟，对突变反应慢

### 2. 新方案：卡尔曼滤波
```
编码器速度 → [卡尔曼滤波器] → 融合速度
    ↓
  加速度  (用于预测)
```

**优势：**
- ✅ **动态加权**：根据实际噪声动态调整信任度
- ✅ **低延迟**：加速度计短期反应快，突变时快速响应
- ✅ **长期精度**：编码器长期累积精度高
- ✅ **最优估计**：理论上最小化滤波误差

## 工作原理

### 卡尔曼滤波流程（每个周期5ms）：

```
1. 预测阶段（使用加速度）：
   v_predict = v_previous + a * dt
   P_predict = P_previous + Q

2. 测量阶段（编码器数据）：
   K = P_predict / (P_predict + R)  // 卡尔曼增益
   v_estimate = v_predict + K * (v_encoder - v_predict)
   P_new = (1 - K) * P_predict

3. 输出：v_estimate → v_x_car, v_y_car
```

## 参数配置

在 `common.h` 中配置：

```c
#define KALMAN_Q_VELOCITY   0.01f   // 过程噪声 (加速度不确定性)
#define KALMAN_R_VELOCITY   10.0f   // 测量噪声 (编码器噪声)
#define KALMAN_P_VELOCITY   1.0f    // 初始误差
```

### 参数调整指南

| 参数 | 含义 | 调大 | 调小 |
|-----|------|------|------|
| **Q** | 加速度的可信度 | 更相信加速度（反应快） | 更相信编码器（精度高） |
| **R** | 编码器的噪声 | 不信任编码器（可能漂移） | 信任编码器（可能滞后） |

#### 调参建议：

**情况1：需要快速响应（如路障躲避，快速加速）**
```c
#define KALMAN_Q_VELOCITY   0.05f   // 增大，更快相信加速度
#define KALMAN_R_VELOCITY   20.0f   // 调大
```

**情况2：需要高精度位置（如停车对位，长距离行驶）**
```c
#define KALMAN_Q_VELOCITY   0.005f  // 减小，更信任编码器
#define KALMAN_R_VELOCITY   5.0f    // 减小，更信任编码器
```

**情况3：中等配置（推荐初始值）**
```c
#define KALMAN_Q_VELOCITY   0.01f   // 默认
#define KALMAN_R_VELOCITY   10.0f   // 默认
```

## 代码变化

### encoder.c
```c
// 改动：分离出 v_x_encoder 和 v_y_encoder
// 不再直接修改 v_x_car 和 v_y_car
v_x_encoder = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) / 4.0f;
v_y_encoder = MECANUM_KY * (-wheel_speed[0] + wheel_speed[1] + wheel_speed[2] - wheel_speed[3]) / 4.0f;
```

### imu.c
```c
// 新增函数：初始化卡尔曼滤波
void velocity_fusion_init(void)
{
    kalman_filter_2d_init(&velocity_filter, 
                          KALMAN_Q_VELOCITY,
                          KALMAN_R_VELOCITY,
                          KALMAN_P_VELOCITY,
                          0.0f, 0.0f);
}

// world_position_get()中使用卡尔曼滤波融合
kalman_filter_2d_update(&velocity_filter, 
                        v_x_encoder, v_y_encoder,  // 编码器测量值
                        acc_x, acc_y,              // 加速度计加速度
                        SENSOR_SOLVE_dt);

// 获取融合后的速度
kalman_filter_2d_get_velocity(&velocity_filter, &v_x_car, &v_y_car);
```

### 文件清单
- `code/inc/kalman_filter.h` - 卡尔曼滤波器头文件
- `code/src/kalman_filter.c` - 卡尔曼滤波器实现
- `code/src/imu.c` - 改进的IMU处理
- `code/src/encoder.c` - 分离编码器速度
- `user/inc/data.h` - 添加滤波器全局变量声明
- `user/src/data.c` - 初始化滤波器实例
- `user/inc/common.h` - 添加卡尔曼参数

## 验证方法

### 1. 测试响应延迟
```
做法：突然加速，观察v_x_car响应时间
预期：应该快速响应（<100ms）
```

### 2. 测试长距离精度
```
做法：走直线50米，对比编码器里程和世界坐标距离
预期：误差 < 5% （相比原方案可能改进5~10%）
```

### 3. 测试噪声抑制
```
做法：静止停车5分钟，观察x_world/y_world的漂移
预期：位置漂移 < 50mm （几乎无法察觉）
```

## 注意事项

1. **加速度计必须标定好**：如果零偏不准，卡尔曼会自动调整，但效果下降
2. **编码器线数和齿轮比必须准确**：这是长期精度的基础
3. **融合初始化**：第一个周期会使用编码器值初始化卡尔曼，之后才逐步融合
4. **参数冻硬在运行时**：不能动态修改Q/R值，需要重编译

## 与原方案的兼容性

- ✅ 完全无缝替代，无需改动控制逻辑
- ✅ 保留所有原有的陀螺仪航向角解算
- ✅ v_x_car/v_y_car输出接口完全兼容
- ✅ 可以随时切换回互补滤波（改回encoder.c即可）

## 后续可能的改进

1. **扩展卡尔曼滤波（EKF）**：加入航向角，3自由度联合滤波
2. **IMU预积分**：加速度计预积分替代简单积分
3. **自适应Q/R**：根据加速度幅度动态调整参数
4. **轮胎滑率检测**：编码器和IMU不一致时警告

---
创建日期：2026年3月5日
