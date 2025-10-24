## 1. 用gazebo将urdf转换为sdf格式

```bash
gz sdf -p miniArm.urdf > miniArm.sdf
```
## 2. gazebo生成的sdf文件碰撞体积丢失问题
### sw导出的urdf用stl文件描述碰撞体积，STL 是“表面网格”，没有封闭性和凸性信息。Gazebo 的 Bullet 或 DART 物理引擎要求碰撞体是凸形或封闭体，否则会出错（例如物体爆炸、穿透、抖动）。
> 直接使用urdf让gz自己解析

### 如果保留原始数据不简化，直接运行仿真会导致仿真崩溃