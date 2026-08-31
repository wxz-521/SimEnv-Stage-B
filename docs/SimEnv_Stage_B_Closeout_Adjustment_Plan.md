# SimEnv 阶段 B 收尾调整方案

> 适用工程：SimEnv  
> 官方仓库：https://gitee.com/guoyulun/SimEnv.git  
> 当前阶段：Stage B 单层四房稳定化收尾  
> 目标：在不继续堆叠机制的前提下，提高四房完成率、危险源检测稳定性、定位异常处理能力，并完成正式收尾验收。  
> 核心原则：**减少停留、减少重复扫描、减少无意义恢复；定位问题归定位层，门问题归门融合层，危险源问题归感知链。**

---

## 1. 当前阶段判断

Stage B 主闭环已经成立，但尚未稳定收尾。

当前真实 Gazebo 多 seed 结果已经证明：

- 单层四房完整闭环可以成功；
- Scan / Map 双门证据融合能够工作；
- 短时 2.5D Door Verifier 已接入；
- FAST-LIO、局部回环、危险源检测、进出房状态机均已运行；
- 但五 seed 中整体通过率仍低；
- 失败并不是同一种原因。

当前必须把问题拆成四条独立链：

```text
B-DET         危险源漏检 / 误报
B-CORRIDOR    走廊推进 / 进度识别停滞
B-FAULT       跌倒 / 点云失效 / 定位异常后的任务状态
B-DOOR        门候选结构与真假门验收
```

禁止继续把所有失败都归结为：

```text
“门检测还不够强”
```

或：

```text
“LIO 还不够准”
```

---

# 2. 本轮最重要的结构调整：取消“第一观察位”的观察职责

当前房间行为中，原 P1 同时承担：

```text
进入房间后的第一观察位
+
360°扫描
+
局部回环锚点
+
危险源轨迹基线
```

这几个职责叠加在一起会带来：

- 房内额外停留；
- 不必要的完整旋转；
- LIO 在狭小、近墙、家具环境下累计更多误差；
- P1 与后续观察点存在观察内容重复；
- 进入每个房间都固定执行一轮扫描，即使真正高价值观察位置还在更里面。

因此调整为：

> **原 P1 不再是 Observation Pose，而改为 Room Anchor。**

建议正式重命名：

```text
P1
↓
ROOM_ANCHOR
```

或：

```text
ENTRY_ANCHOR
```

---

# 3. ROOM_ANCHOR 的新职责

机器人完成进门后，在原 P1 附近短暂停留。

这里只做：

```text
1. 保存当前 FAST-LIO pose
2. 保存门口 / 房间近端局部点云
3. 保存当前 corridor / door normal
4. 保存危险源 tracker 基线
5. 建立 Room Anchor Frame
6. 为退出后的局部回环提供 reference keyframe
```

不再做：

```text
360°视觉扫描
完整 RGB-D 房间搜索
危险源覆盖判断
长时间原地旋转
```

## 3.1 ROOM_ANCHOR 停留时间

应尽可能短。

建议第一版：

```text
0.3 ~ 1.0 sim s
```

只要满足：

```text
局部点云数量达到阈值
+
FAST-LIO 状态 GOOD
+
关键帧成功保存
```

就立即进入主观察阶段。

禁止为了“点云更多一点”固定停留数秒。

---

# 4. 原 P2 升级为 MAIN_OBSERVATION

原第二观察位不再理解成“第二个观察点”。

它应改成：

```text
MAIN_OBSERVATION
```

即：

> **每个房间真正执行危险源搜索的主观察位置。**

新的房间行为：

```text
ENTER_ROOM
    ↓
ROOM_ANCHOR
    ↓
短时保存局部回环锚点
    ↓
MOVE_TO_MAIN_OBSERVATION
    ↓
MAIN_OBSERVATION_SCAN
    ↓
是否发现需要二次确认的新红球？
    │
    ├── NO
    │    ↓
    │  RETURN / EXIT
    │
    └── YES
         ↓
      TARGET_VERIFY
         ↓
      RETURN / EXIT
```

---

# 5. MAIN_OBSERVATION 的位置原则

MAIN_OBSERVATION 不要求等于房间几何中心。

仍然坚持：

> **安全位置优先于几何中心。**

机器人沿门内法向进入。

移动过程中持续检查：

```text
front_clearance
narrow_front_min_range
room_motion_stop_distance
```

如果前方有：

- 家具；
- 红球；
- 其他障碍；
- 深度不可可靠确认的近障碍；

则提前停止。

停止点直接成为：

```text
MAIN_OBSERVATION
```

不再强行前往预设中心点。

---

# 6. MAIN_OBSERVATION 扫描方式

MAIN_OBSERVATION 是主要视觉搜索位置。

建议：

```text
一次完整扫描
```

即可承担原 P1 + P2 的大部分观察职责。

如果当前三扇区扫描已经运行稳定，可以保留：

```text
3 × 90° sector
```

加上进入方向已经覆盖的视场。

原则：

> **避免重复看已经在进入过程中充分观测过的方向。**

不建议再额外：

```text
ROOM_ANCHOR 360°
+
MAIN_OBSERVATION 360°
```

这会造成明显冗余。

---

# 7. 新红球出现时才进入 TARGET_VERIFY

你当前的正确思想继续保留：

> **只有发现新的红色球形候选时，才增加第二视角。**

但不要再把这个点叫 P2。

建议新状态：

```text
TARGET_VERIFY
```

或：

```text
SECOND_VIEW_VERIFY
```

## 7.1 TARGET_VERIFY 的目标

它不是第二次搜索整间房。

只负责：

```text
确认当前新红球
+
获得第二视角
+
改善深度 / 几何确认
+
降低红方块误报
+
减少目标轨迹分裂
```

## 7.2 TARGET_VERIFY 的移动范围

必须严格限制。

建议：

```text
main observation → verify pose
translation <= 0.5 ~ 0.8 m
```

尽量：

```text
< 1.0 m
```

不允许为了寻找更好角度在房间中大范围移动。

## 7.3 TARGET_VERIFY 不执行完整 360°

只针对当前目标方向进行：

```text
定向观察
```

例如：

```text
目标方位 ±45°
```

或：

```text
90° ~ 180° 局部转动
```

完成确认后立即结束。

---

# 8. 房间危险源坐标先锚定到 Room Frame

当前房间内存在：

```text
FAST-LIO 平滑累计漂移
```

尤其：

```text
旋转
+
短距离移动
+
第二视角
```

以后容易导致同一危险源 world 坐标出现偏差。

因此建议：

```text
Camera
↓
Base
↓
RoomAnchorFrame
↓
P_room
```

房间内先保存：

```text
P_room
```

而不是每一帧都只依赖当前 LIO 立即生成最终：

```text
P_world
```

## 8.1 危险源轨迹关联

MAIN_OBSERVATION 第一次发现目标时：

```text
create TargetHypothesis
```

保存：

```text
target_id
P_room
bearing
depth
shape evidence
observation_count
```

TARGET_VERIFY 看到目标时：

```text
associate to existing TargetHypothesis
```

而不是重新以 world 距离建立一个全新轨迹。

这样可以降低：

```text
同一红球
↓
因 LIO 局部漂移
↓
拆成两条 world track
```

的问题。

---

# 9. 退出后的同门局部回环继续保留

当前同门局部回环已经证明：

- 多轮被接受；
- 正常修正量较小；
- 不是 `20260825` 74 m 跳变的来源。

因此：

> **局部回环暂时冻结，不扩大、不增加新的全局回环机制。**

房间退出后：

```text
current door / room local cloud
↓
match ROOM_ANCHOR keyframe
↓
bounded ICP
↓
accepted / rejected
```

## 9.1 局部回环暂时只承担两项职责

第一：

```text
估计本次房间访问的局部漂移
```

第二：

```text
修正 Room Frame 下危险源最终 world 坐标
```

暂时不要：

```text
大幅直接修改 FAST-LIO 内部状态
```

避免产生 TF jump。

---

# 10. 新的房间状态机

调整后建议：

```text
GO_TO_PRE_DOOR
    ↓
ALIGN_TO_DOOR_NORMAL
    ↓
DOOR_CROSSING
    ↓
ROOM_ANCHOR
    ↓
MOVE_TO_MAIN_OBSERVATION
    ↓
MAIN_OBSERVATION_SCAN
    │
    ├── no new target
    │       ↓
    │     EXIT_ROOM
    │
    └── new target
            ↓
       TARGET_VERIFY
            ↓
         EXIT_ROOM
            ↓
   DOOR_LOCAL_LOOP
            ↓
        VISITED
```

注意：

```text
ROOM_ANCHOR
```

不是观察位。

---

# 11. B-CORRIDOR：专项解决 `20260824`

`20260824` 是当前最干净的导航失败案例。

特点：

```text
FAST-LIO GOOD
最大位姿步长正常
RTF 正常
已完成两进两出
局部回环修正很小
然后走廊推进停滞
WAITING_FOR_OPENINGS 长时间持续
```

这一问题暂时禁止修改：

```text
门宽阈值
2.5D Verifier
FAST-LIO
局部回环
危险源参数
```

## 11.1 只增加诊断

每周期记录：

```text
sim_time
state

cmd_vel.linear.x
cmd_vel.angular.z

LIO pose
delta_translation
delta_yaw

corridor_progress_start
corridor_progress_current
requested_progress

front_clearance
left_wall_distance
right_wall_distance
corridor_valid
corridor_confidence

finite_scan_count

scan_door_state
OPEN_START
OPEN
OPEN_END

DoorEvidence count
DoorHypothesis count
```

## 11.2 必须区分三种根因

### A. 命令在发，机器人没有实际移动

归类：

```text
LOCOMOTION / OBSTACLE
```

检查：

- RL controller；
- 足端执行；
- 前方障碍；
- 贴墙；
- 局部碰撞。

### B. 机器人实际移动，但 LIO progress 不足

归类：

```text
LOCALIZATION METRIC
```

检查：

- FAST-LIO 纵向平移；
- 走廊退化；
- progress 使用的坐标；
- correction 是否重复。

### C. LIO 位移正常，但状态机 progress 不增加

归类：

```text
STATE MACHINE BOOKKEEPING
```

检查：

- `corridor_progress_start` 更新；
- state transition；
- reverse sweep；
- reset 条件。

只有完成这个分类以后才能修改代码。

---

# 12. B-FAULT：专项解决 `20260825`

`20260825` 不应视为普通 LIO 漂移。

现有证据更符合：

```text
机器人跌倒
↓
controller passive/down
↓
点云 No Effective Points
↓
FAST-LIO 异常
↓
74 m 非物理 pose jump
↓
Localization BAD
↓
状态机继续等待
```

因此新增：

```text
MISSION_FAULT
```

## 12.1 Fault 触发条件

任一满足：

```text
controller passive/down
```

或：

```text
fall_detected
```

或：

```text
LocalizationHealth == BAD 持续超过阈值
```

或：

```text
effective_lidar_points 连续过低
```

或：

```text
nonphysical_pose_jump
```

则：

```text
MISSION_FAULT
```

## 12.2 Fault 后行为

立即：

```text
zero cmd_vel
停止创建 DoorEvidence
停止更新正式 danger track
停止 corridor timeout
停止 WAITING_FOR_OPENINGS
```

然后：

```text
可恢复
→ RECOVERY

不可恢复
→ ABORT_RUN
```

禁止一轮已经物理失败以后继续运行 100~200 秒污染统计。

---

# 13. B-DET：危险源检测收尾

危险源仍然是当前通过率最低的一项。

但必须严格区分：

```text
没有访问危险源房间
```

和：

```text
访问了房间但 detector 漏检
```

只有：

```text
房间实际访问完成
```

以后，才统计真正 detector recall。

## 13.1 增加完整检测生命周期日志

每个红色候选记录：

```text
RED_MASK_FOUND
↓
CONTOUR_PASS / REJECT
↓
DEPTH_PASS / REJECT
↓
TF_PASS / REJECT
↓
TRACK_CREATED
↓
TRACK_ASSOCIATED
↓
OBSERVATION_COUNT
↓
CONFIRMED / UNCONFIRMED
↓
LOCAL_LOOP_CORRECTION
↓
FINAL_OUTPUT
```

任何漏检最终必须能够落到其中一个阶段。

## 13.2 暂时不要先改 HSV

在没有证明：

```text
RED_MASK_FOUND == false
```

之前，不先扩大红色 HSV 阈值。

否则可能提高红方块误报。

---

# 14. B-DOOR：门结构问题

最新复测已经出现：

```text
VISITED=4
4进4出
Danger正确
LIO GOOD
```

但：

```text
door_structure=unpaired_sides
```

因此当前必须先确认：

> 生成器是否真的保证每层严格两左两右配对房间。

如果生成器并没有这个结构硬约束，则：

```text
unpaired_sides
```

不能作为正式失败条件。

如果生成器确实保证，则继续查：

```text
漏掉了哪一个真门
+
哪个同侧候选是假门
```

## 14.1 2.5D Verifier 暂时不升级

当前短时 2.5D Door Verifier 已经存在。

暂时不改成：

```text
全局持续 2.5D mapping
```

因为当前没有证据证明门结构失败是由于 2.5D 信息不足。

继续维持：

```text
Scan / Map
↓
DoorFusion
↓
short-time 2.5D verification
```

即可。

---

# 15. Stage B 暂时冻结的机制

以下部分除非出现直接证据，否则暂时禁止调整：

```text
FAST-LIO 主参数
同门局部 ICP 阈值
Scan / Map 双门基本结构
2.5D Door Verifier 基本结构
门宽 0.8~1.6 m 范围
defer zone
entrance gate
```

当前重点不是再次改总体架构。

---

# 16. 本轮建议的代码调整优先级

## Priority 1：房间状态机减法

首先完成：

```text
P1 → ROOM_ANCHOR
取消 ROOM_ANCHOR 360° scan
原 P2 → MAIN_OBSERVATION
新红球 → TARGET_VERIFY
```

这是低风险、高收益调整。

## Priority 2：增加 Fault State

完成：

```text
fall / passive / bad localization / no effective points
→ MISSION_FAULT
```

防止异常轮次继续运行。

## Priority 3：补 `20260824` 诊断

只增加日志。

先不修改算法。

明确：

```text
机器人没移动
/
LIO没累计
/
状态机没认进度
```

## Priority 4：危险源生命周期日志

明确每次误报 / 漏检到底发生在：

```text
颜色
形状
深度
TF
轨迹
确认
回环
```

哪一级。

## Priority 5：验证 door_structure 规则

确认：

```text
unpaired_sides
```

是有效结构约束还是监控器过度假设。

---

# 17. 新 Stage B 验收条件

至少连续 5 个正式 seed 满足：

```text
4/4 rooms
floor_complete=true
4进4出
UNREACHABLE=0
无额外假门
danger 无漏检
danger 无红方块误报
最大非物理 pose jump < 1m
无人工干预
无定位 crash
无持续 fault 后继续探索
```

同时：

```text
ROOM_ANCHOR
```

必须只作为局部回环锚点，不再执行完整房间观察。

---

# 18. Stage B 收尾后的最终单房流程

推荐固定为：

```text
CORRIDOR
   ↓
DOOR_FOUND
   ↓
GO_TO_PRE_DOOR
   ↓
ALIGN
   ↓
CROSS_DOOR
   ↓
ROOM_ANCHOR
   │
   ├── save pose
   ├── save local cloud
   ├── save tracker baseline
   └── short dwell only
   ↓
MOVE_TO_MAIN_OBSERVATION
   ↓
MAIN_OBSERVATION_SCAN
   │
   ├── no new red target
   │       ↓
   │      EXIT
   │
   └── new red target
           ↓
      TARGET_VERIFY
           ↓
          EXIT
           ↓
      LOCAL_LOOP
           ↓
      VISITED
```

---

# 19. 最终原则

阶段 B 后续不再以：

```text
“再增加一个机制”
```

作为默认解决问题的方法。

必须坚持：

> **P1 只做锚点，不再重复观察。**

> **真正的房间搜索集中到一个 MAIN_OBSERVATION。**

> **只有发现新红球才增加 TARGET_VERIFY。**

> **20260824、20260825、危险源检测和门结构问题分别处理，禁止混调。**

> **跌倒和定位失效进入 Fault，而不是继续等待。**

> **局部回环当前已经够用，暂时不升级成全局回环。**

> **Stage B 收尾目标是稳定，而不是继续扩展功能。**
