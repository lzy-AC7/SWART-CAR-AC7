# 卡尔曼滤波器参数调参详解

## 📊 参数含义

### 1. KALMAN_Q_VELOCITY (过程噪声方差)
```c
#define KALMAN_Q_VELOCITY       0.01f   // 过程噪声方差（加速度不确定性）
```

**物理含义**：
- 表示加速度计的测量不确定性
- 越大表示加速度计噪声越大，越不相信加速度计
- 越小表示加速度计很准，越相信加速度计

**影响效果**：
- **Q 增大** → 更相信编码器，反应慢但稳定
- **Q 减小** → 更相信加速度计，反应快但可能振荡

---

### 2. KALMAN_R_VELOCITY (测量噪声方差)
```c
#define KALMAN_R_VELOCITY       10.0f   // 测量噪声方差（编码器噪声）
```

**物理含义**：
- 表示编码器的测量不确定性
- 越大表示编码器噪声越大，越不相信编码器
- 越小表示编码器很准，越相信编码器

**影响效果**：
- **R 增大** → 不相信编码器，滤波器更依赖加速度计预测
- **R 减小** → 相信编码器，滤波器更依赖编码器测量

---

### 3. KALMAN_P_VELOCITY (初始估计误差方差)
```c
#define KALMAN_P_VELOCITY       1.0f    // 初始估计误差方差
```

**物理含义**：
- 表示初始时刻对速度估计的不确定性
- 越大表示初始不确定性大，滤波器更容易被新测量影响
- 越小表示初始很确定，滤波器更保守

**影响效果**：
- **P 增大** → 初始更容易相信新数据，收敛快但可能不稳定
- **P 减小** → 初始很保守，收敛慢但更稳定

## 🎯 调参策略

### 原则：Q 和 R 的相对大小决定信任度

```
信任加速度计多 → Q 小，R 大
信任编码器多   → Q 大，R 小
```

### 场景1：需要快速响应（竞速、躲障）
```c
#define KALMAN_Q_VELOCITY       0.005f   // 减小Q，更相信加速度计
#define KALMAN_R_VELOCITY       15.0f    // 增大R，不太相信编码器
#define KALMAN_P_VELOCITY       1.5f     // 稍大，快速收敛
```
**特点**：加速快，反应灵敏，但可能有抖动

---

### 场景2：需要高精度（长距离、停车）
```c
#define KALMAN_Q_VELOCITY       0.02f    // 增大Q，更相信编码器
#define KALMAN_R_VELOCITY       5.0f     // 减小R，很相信编码器
#define KALMAN_P_VELOCITY       0.5f     // 稍小，更保守
```
**特点**：稳定精确，位置误差小，但反应稍慢

---

### 场景3：平衡配置（推荐默认）
```c
#define KALMAN_Q_VELOCITY       0.01f    // 中等
#define KALMAN_R_VELOCITY       10.0f    // 中等
#define KALMAN_P_VELOCITY       1.0f     // 中等
```
**特点**：各方面平衡，适合大多数场景

## 🔍 调参步骤

### 第1步：确定应用场景
- **竞速比赛** → 选择场景1
- **长距离导航** → 选择场景2
- **通用应用** → 保持场景3

### 第2步：观察实际表现
在 `debug.c` 中添加监测：
```c
// 观察融合效果
justfloat_add(4, v_x_encoder, v_x_car, acc_x, omega_car);
// v_x_encoder: 编码器速度（基准）
// v_x_car: 融合后速度（输出）
// acc_x: 加速度计数据
// omega_car: 角速度
```

### 第3步：分析波形特征

#### 理想波形应该：
```
v_x_car ≈ v_x_encoder 的平滑版本
        + 对 acc_x 的快速响应
        - 不过度放大噪声
```

#### 问题识别：
- **v_x_car 跟随 v_x_encoder 太慢** → 减小 Q 或增大 R
- **v_x_car 抖动太大** → 增大 Q 或减小 R
- **加速时反应迟钝** → 减小 Q
- **静止时位置漂移** → 增大 Q

### 第4步：微调参数

#### 快速调参表：

| 问题现象 | 可能原因 | 调整方案 |
|---------|---------|---------|
| 加速反应慢 | Q 太大 | Q ÷ 2 |
| 速度抖动大 | Q 太小 | Q × 2 |
| 位置不准 | R 太大 | R ÷ 2 |
| 过于保守 | R 太小 | R × 2 |
| 启动收敛慢 | P 太小 | P × 1.5 |
| 启动振荡 | P 太大 | P ÷ 1.5 |

## 📈 数学原理

### 卡尔曼增益计算：
```
K = P预测 / (P预测 + R)
```

- **K ≈ 1**：主要相信测量值（编码器）
- **K ≈ 0**：主要相信预测值（加速度计）
- **K ≈ 0.5**：均衡相信两者

### 参数影响：
- **增大 Q** → P预测增大 → K增大 → 更相信测量
- **增大 R** → K减小 → 更相信预测
- **增大 P** → 初始K较大 → 更快收敛

## 🧪 实际测试方法

### 测试1：阶跃响应测试
```
做法：突然给一个速度指令，观察 v_x_car 的响应时间
预期：< 200ms 达到90%目标值
```

### 测试2：稳态精度测试
```
做法：匀速直线行驶，观察位置误差累积
预期：< 2% 距离误差
```

### 测试3：噪声抑制测试
```
做法：静止状态，观察 v_x_car 的波动幅度
预期：< 5% 最大速度波动
```

## 💡 经验参数值

### 不同车速下的推荐值：

#### 低速（< 1 m/s）
```c
#define KALMAN_Q_VELOCITY       0.02f    // 增大Q，相信编码器
#define KALMAN_R_VELOCITY       8.0f     // 中等
#define KALMAN_P_VELOCITY       0.8f     // 稍保守
```

#### 中速（1-3 m/s）
```c
#define KALMAN_Q_VELOCITY       0.01f    // 默认
#define KALMAN_R_VELOCITY       10.0f    // 默认
#define KALMAN_P_VELOCITY       1.0f     // 默认
```

#### 高速（> 3 m/s）
```c
#define KALMAN_Q_VELOCITY       0.008f   // 稍小，更相信加速度计
#define KALMAN_R_VELOCITY       12.0f    // 稍大，编码器高速时噪声大
#define KALMAN_P_VELOCITY       1.2f     // 稍大
```

## ⚠️ 注意事项

1. **参数是成对调整的**：Q 和 R 需要一起考虑
2. **不要极端化**：Q 或 R 不要相差超过10倍
3. **测试环境重要**：在实际赛道上测试，不要只在实验室
4. **记录参数**：每次调整后记录参数和表现
5. **逐步调整**：每次只改一个参数，观察效果

## 🔄 快速切换方案

可以在代码中准备多套参数，通过条件编译切换：

```c
// 在 common.h 中
#define KALMAN_MODE 1  // 0=默认, 1=快速响应, 2=高精度

#if KALMAN_MODE == 0
    #define KALMAN_Q_VELOCITY       0.01f
    #define KALMAN_R_VELOCITY       10.0f
    #define KALMAN_P_VELOCITY       1.0f
#elif KALMAN_MODE == 1
    #define KALMAN_Q_VELOCITY       0.005f
    #define KALMAN_R_VELOCITY       15.0f
    #define KALMAN_P_VELOCITY       1.5f
#else
    #define KALMAN_Q_VELOCITY       0.02f
    #define KALMAN_R_VELOCITY       5.0f
    #define KALMAN_P_VELOCITY       0.5f
#endif
```

---

**总结**：从默认参数开始，根据具体应用场景和测试结果逐步调整。记住，调参是一个迭代过程，需要结合实际测试数据！