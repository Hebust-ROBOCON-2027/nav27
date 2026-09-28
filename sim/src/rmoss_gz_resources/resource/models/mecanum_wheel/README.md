# Mecanum Wheel Model

麦克纳姆轮模型，包含简化的碰撞几何体。

## 特性

- **Visual（视觉）**：使用详细的 DAE mesh，保证外观真实
- **Collision（碰撞）**：使用简化的圆柱体，避免 ODE trimesh 溢出，提升性能

## 参数

- 半径：0.0758m
- 宽度：0.08m
- 质量：1.0kg
- 摩擦系数：μ=0.8

## 使用方法

### 方法 1：在 SDF 中引用模型

```xml
<include>
  <uri>model://mecanum_wheel</uri>
  <name>my_wheel</name>
  <pose>0 0 0 0 0 0</pose>
</include>
```

### 方法 2：在 xmacro 中使用

可以修改 `AT_R2.sdf.xmacro` 中的轮子定义，引用此模型。

## 左右轮差异

如需左轮和右轮，可以在 visual 的 uri 中指定：
- 左轮：`model://rmua19_standard_robot/meshes/wheel_left01.dae`
- 右轮：`model://rmua19_standard_robot/meshes/wheel_right01.dae`
