# 覆盖路径规划服务 API 文档

## 概述

本文档描述了覆盖路径规划服务的HTTP API接口，该服务基于Fields2Cover库实现，为农业机器人提供智能覆盖路径规划功能。服务运行在端口1235，提供完整的字段覆盖路径规划和导航控制功能。

**服务地址**: `http://服务器IP:1235`

## 主要功能

- 🌾 **智能覆盖规划**: 基于字段边界自动生成最优覆盖路径
- 🗺️ **地图集成**: 支持全局代价地图分析和障碍物检测
- 📐 **字段分割**: 支持按最大面积自动分割大字段
- 🎯 **精确导航**: 生成高精度机器人导航路点
- 📊 **可视化输出**: 自动生成字段、路径、障碍物可视化图像

---

## API 接口详情

### 1. 覆盖路径规划接口

**接口名称**: 生成覆盖路径规划  
**请求方式**: `POST`  
**请求地址**: `/navigate_coverage`  
**Content-Type**: `application/json`

#### 请求参数

| 参数名 | 类型 | 必填 | 描述 | 默认值 | 示例 |
|--------|------|------|------|--------|------|
| `field` | Array | ✅ | 字段边界坐标点数组，每个点为[x, y]格式 | - | `[[0,0], [100,0], [100,50], [0,50]]` |
| `use_user_field` | Boolean | ❌ | 是否使用用户提供的字段边界 | `true` | `false` |
| `mode` | String | ❌ | 路径规划模式 | `SET_ANGLE` | `BRUTE_FORCE` |
| `best_angle` | Number | ❌ | 覆盖角度（度数） | `0.0` | `45.0` |
| `step_angle` | Number | ❌ | 角度步长（度数） | `0.2` | `5.0` |
| `objective` | String | ❌ | 优化目标 | `COVERAGE` | `LENGTH` |
| `rings` | Array | ❌ | 字段内部障碍物环形边界 | `[]` | `[[[10,10], [20,10], [20,20], [10,20]]]` |
| `robot_width` | Number | ❌ | 机器人宽度（米） | `1.0` | `1.5` |
| `robot_op_width` | Number | ❌ | 机器人作业宽度（米） | `1.0` | `2.0` |
| `robot_min_turning_radius` | Number | ❌ | 最小转弯半径（米） | `0.00000001` | `1.5` |
| `robot_max_diff_curvature` | Number | ❌ | 最大曲率差 | `100000000` | `1000000` |
| `robot_cruise_vel` | Number | ❌ | 巡航速度（米/秒） | `0.5` | `1.0` |
| `robot_turn_vel` | Number | ❌ | 转弯速度（米/秒） | `0.5` | `0.3` |
| `use_decomposition` | Boolean | ❌ | 是否使用字段分解 | `true` | `false` |
| `decomposition_type` | String | ❌ | 分解类型 | `Trapezoidal` | `Boustrophedon` |
| `headland_width` | Number | ❌ | 地头宽度（米） | `1.0` | `3.0` |
| `swath_allow_overlap` | Boolean | ❌ | 是否允许路径重叠 | `false` | `true` |
| `route_mode` | String | ❌ | 路径模式 | `AUTO` | `BOUSTROPHEDON` |
| `route_start` | Array | ❌ | 路径起点坐标[x, y] | `null` | `[0, 0]` |
| `route_spiral` | Number | ❌ | 螺旋模式参数 | `2` | `3` |
| `route_custom_order` | Array | ❌ | 自定义路径顺序 | `[]` | `[0, 2, 1, 3]` |
| `repeat_times` | Number | ❌ | 重复次数 | `1` | `3` |

#### 参数详细说明

##### 路径规划模式 (`mode`)
- `SET_ANGLE`: 固定角度模式，使用`best_angle`指定的角度
- `BRUTE_FORCE`: 暴力搜索模式，尝试多个角度找到最优解

##### 优化目标 (`objective`)
- `COVERAGE`: 优化覆盖率
- `LENGTH`: 优化路径长度
- `NUMBER`: 优化路径数量

##### 路径模式 (`route_mode`)
- `AUTO`: 自动选择最优路径
- `BOUSTROPHEDON`: 往复式路径
- `SNAKE`: 蛇形路径  
- `SPIRAL`: 螺旋路径
- `CUSTOM`: 自定义顺序路径

##### 分解类型 (`decomposition_type`)
- `Trapezoidal`: 梯形分解
- `Boustrophedon`: 往复分解

#### 请求示例

```json
{
  "field": [
    [10.0, 20.0],
    [50.0, 20.0], 
    [50.0, 60.0],
    [10.0, 60.0]
  ],
  "use_user_field": false,
  "mode": "BRUTE_FORCE",
  "best_angle": 0,
  "step_angle": 5.0,
  "objective": "COVERAGE",
  "rings": [],
  "robot_width": 1.0,
  "robot_op_width": 2.0,
  "robot_min_turning_radius": 1.5,
  "headland_width": 2.0,
  "route_mode": "AUTO",
  "route_start": [15.0, 25.0],
  "repeat_times": 1
}
```

#### 响应格式

**成功响应** (HTTP 202):
```json
{
  "code": 0,
  "path": [
    {
      "x": 15.5,
      "y": 25.3,
      "z": 0.0
    },
    {
      "x": 45.2,
      "y": 25.3,
      "z": 0.0
    }
  ],
  "status": "导航任务已启动"
}
```

**错误响应** (HTTP 400/404/500):
```json
{
  "code": 1,
  "error": "错误描述信息"
}
```

#### 常见错误码

| 错误码 | HTTP状态码 | 描述 |
|--------|-----------|------|
| 1 | 400 | 请求参数错误 |
| 1 | 404 | 未找到可用自由空间 |
| 1 | 500 | 服务器内部错误 |

---

### 2. 导航执行接口

**接口名称**: 开始导航执行  
**请求方式**: `POST`  
**请求地址**: `/start_navigation`  
**Content-Type**: `application/json`

#### 请求参数

| 参数名 | 类型 | 必填 | 描述 | 默认值 |
|--------|------|------|------|--------|
| `wait_at_first_waypoint` | Boolean | ❌ | 是否在第一个路点等待 | `false` |
| `path_planner` | String | ❌ | 路径规划器类型 | `straight` |
| `precise_xy` | Number | ❌ | XY精度要求（米） | `0.1` |
| `inflation_radius` | Number | ❌ | 膨胀半径（米） | `0.1` |

#### 请求示例

```json
{
  "wait_at_first_waypoint": true,
  "path_planner": "straight",
  "precise_xy": 0.05,
  "inflation_radius": 0.15
}
```

#### 响应格式

**成功响应** (HTTP 200):
```json
{
  "code": 0,
  "status": "导航任务已开始"
}
```

**错误响应** (HTTP 400/500):
```json
{
  "code": 1,
  "error": "错误描述"
}
```

---

## 可视化输出文件

服务会自动生成以下可视化图像文件，可通过HTTP下载：

### 字段可视化文件

| 文件名 | 描述 |
|--------|------|
| `user_provided_field.png` | 用户提供的字段边界可视化 |
| `cropped_free_space.png` | 裁剪后的自由空间可视化 |
| `user_provided_field_with_routes.png` | 带覆盖路径的用户字段 |
| `cropped_free_space_with_routes.png` | 带覆盖路径和障碍物的自由空间 |

### 代价地图文件

| 文件名 | 描述 |
|--------|------|
| `received_costmap_YYYYMMDD_HHMMSS.png` | 带元数据的代价地图 |
| `received_costmap_YYYYMMDD_HHMMSS_raw.png` | 原始代价地图 |
| `global_costmap_with_divided_areas.png` | 带分割区域的全局代价地图 |
| `cropped_costmap_analysis.png` | 裁剪代价地图分析 |

### Fields2Cover输出

| 文件名 | 描述 |
|--------|------|
| `Tutorial_image.png` | Fields2Cover库生成的覆盖路径可视化 |

---

## 使用流程

### 标准使用流程

1. **🔧 准备字段数据**
   - 确定字段边界坐标（GPS或局部坐标系）
   - 标识字段内障碍物位置（可选）

2. **📡 调用覆盖规划接口**
   ```bash
   POST /navigate_coverage
   # 传入字段边界和规划参数
   ```

3. **📋 获取规划结果**
   - 接收生成的路点序列
   - 下载可视化图像进行验证

4. **🚀 执行导航任务**
   ```bash
   POST /start_navigation  
   # 开始机器人导航执行
   ```

### 高级功能使用

#### 基于代价地图的智能规划
```json
{
  "field": [...],
  "use_user_field": false  // 使用代价地图自动提取自由空间
}
```

---

## 错误处理

### 常见错误及解决方案

| 错误信息 | 原因 | 解决方案 |
|----------|------|----------|
| "缺少'field'字段" | 请求中未包含field参数 | 确保请求包含有效的field坐标数组 |
| "'field'必须是至少包含3个坐标点的列表" | 字段边界点数不足 | 提供至少3个有效边界点 |
| "在用户指定区域内未找到可用的自由空间" | 代价地图中无可通行区域 | 检查字段边界是否在地图范围内 |
| "没有可用的导航路径" | 未先调用覆盖规划接口 | 先调用/navigate_coverage生成路径 |

### 调试建议

1. **📊 查看可视化文件**: 检查生成的PNG文件验证规划结果
2. **📝 查看服务日志**: 观察控制台输出了解详细错误信息  
3. **🗺️ 验证坐标系**: 确保字段坐标与机器人坐标系一致
4. **⚙️ 调整参数**: 根据具体场景优化机器人和规划参数

---

## 配置说明

### 环境要求

- **Python 3.10+**
- **ROS 2** (Humble/Iron)
- **Fields2Cover v1.2.1**
- **OpenCV 4.x**
- **NumPy 1.21+**

### 服务配置

服务默认配置：
- **监听端口**: 1235
- **代价地图服务**: http://127.0.0.1:1234
- **工作目录**: 当前执行目录
- **日志级别**: INFO

### 性能参数

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| 字段面积 | < 10000 m² | 单次规划推荐最大面积 |
| 边界点数 | < 100个点 | 避免过度复杂的边界 |
| 机器人宽度 | 0.5-3.0 m | 根据实际设备调整 |
| 转弯半径 | 0.5-5.0 m | 根据机器人机械限制调整 |

---

## 版本信息

- **API版本**: v1.0
- **最后更新**: 2025年9月
- **维护状态**: 活跃开发中

## 技术支持

如遇到技术问题，请提供：
1. 完整的HTTP请求参数
2. 服务器响应内容  
3. 生成的可视化文件
4. 服务器日志输出

---

## 示例代码

### Android HTTP 请求示例

```java
// 覆盖路径规划请求
public void requestCoveragePlanning() {
    JSONObject requestBody = new JSONObject();
    try {
        // 字段边界
        JSONArray field = new JSONArray();
        field.put(new JSONArray(Arrays.asList(10.0, 20.0)));
        field.put(new JSONArray(Arrays.asList(50.0, 20.0)));
        field.put(new JSONArray(Arrays.asList(50.0, 60.0)));
        field.put(new JSONArray(Arrays.asList(10.0, 60.0)));
        
        requestBody.put("field", field);
        requestBody.put("use_user_field", false);
        requestBody.put("mode", "BRUTE_FORCE");
        requestBody.put("objective", "COVERAGE");
        requestBody.put("rings", new JSONArray());
        requestBody.put("robot_width", 1.5);
        requestBody.put("robot_op_width", 2.0);
        
        // 发送HTTP POST请求
        sendPostRequest("http://服务器IP:1235/navigate_coverage", requestBody);
        
    } catch (JSONException e) {
        e.printStackTrace();
    }
}

// 开始导航请求  
public void startNavigation() {
    JSONObject requestBody = new JSONObject();
    try {
        requestBody.put("wait_at_first_waypoint", true);
        requestBody.put("path_planner", "straight");
        requestBody.put("precise_xy", 0.05);
        
        sendPostRequest("http://服务器IP:1235/start_navigation", requestBody);
        
    } catch (JSONException e) {
        e.printStackTrace();
    }
}
```

---

*本文档为覆盖路径规划服务的完整API说明，如有疑问请联系技术支持团队。*