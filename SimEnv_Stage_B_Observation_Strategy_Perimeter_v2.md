# SimEnv Stage B 房间观察策略新版方案
## —— 主观察位 + 动态进出观察 + 渐进式外围补偿

> 适用阶段：Stage B 单层四房收尾  
> 文档范围：仅讨论房间内部观察策略，不涉及门洞检测、走廊推进、FAST-LIO 参数、楼梯、多楼层与全局任务调度。  
> 核心目标：**危险源不漏检优先，同时避免每个房间固定绕行一圈造成额外路径、碰撞风险和 LIO 累计误差。**

---

# 1. 方案结论

Stage B 房间观察策略不采用：

```text
固定 P1
+
固定 P2
+
每个房间固定完整绕行一圈
```

而采用：

```text
ROOM_ANCHOR
    ↓
INGRESS_SCAN
    ↓
MAIN_OBSERVATION
    ↓
MAIN 360° SCAN
    ↓
PERIMETER_BLIND_CHECK
    │
    ├── 无明显盲区
    │      ↓
    │    EXIT
    │
    └── 存在明显盲区
           ↓
    PARTIAL_PERIMETER
           ↓
      盲区是否仍存在？
        │
      ┌─┴─┐
      │   │
     NO  YES
      │   │
      │ EXTENDED_SWEEP
      │   │
      └─┬─┘
        ↓
      EXIT
        ↓
   EGRESS_SCAN
        ↓
    LOCAL_LOOP
        ↓
      VISITED
```

核心原则：

> **默认不绕房间一圈。先利用已有运动轨迹和一个主观察位完成绝大多数搜索，只有存在明确外围盲区时才逐级增加外围观察。**

---

# 2. 为什么不把“完整绕房一圈”作为默认动作

完整绕房理论上能提高可见区域，但会同时带来：

- 房内移动距离显著增加；
- FAST-LIO 局部累计误差增加；
- 家具和墙之间狭窄区域的卡住风险增加；
- 状态机需要处理更多沿墙、拐角、回退和恢复；
- 不同 seed 下家具布局不同，外围环路未必始终连通；
- 大量已经从 MAIN 可见的区域会被重复观察；
- Stage B 容易重新退化成房间内部覆盖式探索。

因此：

> **完整绕行只作为最高一级 fallback，不作为正常房间的标准流程。**

---

# 3. ROOM_ANCHOR：只做锚点，不做观察

原第一观察位正式调整为：

```text
ROOM_ANCHOR
```

它不是视觉观察位。

---

## 3.1 位置

机器人完全穿过房门后，在门内近端建立 Anchor。

位置要求：

- 机身完全脱离门洞；
- 能稳定看到门框和房间近端局部几何；
- 前方仍有安全空间；
- 离门不远；
- 不追求房间中心。

---

## 3.2 职责

只负责：

```text
保存 LIO pose
保存局部点云
保存门法向
建立 Room Frame
保存危险源 tracker baseline
为退出后的同门局部回环提供 reference
```

---

## 3.3 明确取消

ROOM_ANCHOR 不再执行：

```text
360°扫描
完整 RGB-D 搜索
长时间停留
危险源覆盖判断
```

满足：

```text
LocalizationHealth == GOOD
AND Anchor keyframe 保存成功
AND 局部点云达到最低要求
```

立即离开。

---

# 4. INGRESS_SCAN：把“进入房间”本身变成观察过程

从：

```text
DOOR
→ ROOM_ANCHOR
→ MAIN_OBSERVATION
```

整个过程中：

```text
DangerDetector = ON
```

持续进行：

- RGB 红色候选检测；
- Depth 有效性检查；
- 初始危险源 Track 建立；
- 房间近门区域观察。

---

## 4.1 价值

INGRESS_SCAN 几乎不增加额外运动成本，却可以补充：

- 门口附近；
- 房间前半区域；
- 靠近门两侧的目标；
- 从入口角度能看到的外围自由带。

因此不再把“只有停下来才算观察”。

---

# 5. MAIN_OBSERVATION：唯一固定主观察位

每个房间只固定设置一个真正的观察位置：

```text
MAIN_OBSERVATION
```

承担绝大多数危险源搜索。

---

## 5.1 位置选择

不使用固定几何中心。

优先选择：

> **门口至房间中心之间的前中部安全区域。**

大致目标是：

```text
较大的视野
+
足够净空
+
较短进退路径
+
较少 LIO 累计运动
```

---

## 5.2 动态提前停止

机器人沿门内法向前往理想 MAIN 区域。

持续检查：

```text
front_clearance
narrow_front_min_range
room_motion_stop_distance
```

如果出现：

- 家具；
- 红球；
- 干扰物；
- 近距离障碍；
- 前方通道变窄；

则：

```text
立即停止
→ 当前安全位置直接作为 MAIN_OBSERVATION
```

禁止为了达到固定坐标继续顶着障碍前进。

---

# 6. MAIN 必须完成一次完整 360° 有效扫描

上一版“进入方向已经覆盖，因此 MAIN 只补 3×90°”不再采用。

原因：

机器人从门口进入时，相机主要朝房间内部，无法保证真正覆盖：

```text
门内左后方
门内右后方
门口附近墙边
家具遮挡后的外围区域
```

因此 MAIN 到位以后必须执行：

```text
完整 360° 有效扫描
```

---

## 6.1 扫描方式

建议连续低速旋转一周，而不是：

```text
每 90°
停住
等待
再转
```

逻辑上仍划分为：

```text
Sector 0 :   0°~90°
Sector 1 :  90°~180°
Sector 2 : 180°~270°
Sector 3 : 270°~360°
```

---

## 6.2 扇区完成条件

优先依据有效数据，而不是固定时间：

```text
有效 RGB 帧数
+
有效 Depth 帧数
+
TF 成功
+
最低有效观察数量
```

完成一个扇区后继续下一扇区。

---

# 7. MAIN 之后必须做 PERIMETER_BLIND_CHECK

MAIN 360° 完成以后，不立即假设整个房间已充分观察。

重点判断：

> **是否存在因为家具遮挡而从 MAIN 完全不可见的外围自由区域。**

尤其关注：

```text
门口左侧外围
门口右侧外围
左右墙边自由带
中央家具后方
房间远端外围区域
```

---

# 8. 什么叫“外围盲区”

典型情况：

```text
┌────────────────────────┐
│ ○ danger               │
│                        │
│ █████████ furniture    │
│ █████████              │
│                        │
│          ● MAIN        │
│                        │
└──────── door ──────────┘
```

MAIN 虽然完成 360°，但家具挡住了危险源。

此时要判断：

```text
近距离障碍存在
+
障碍后方仍有明显自由/未知空间
```

如果成立：

```text
PERIMETER_BLIND_ZONE = TRUE
```

---

# 9. PERIMETER_BLIND_CHECK 的第一版实现

不要构建复杂完整覆盖率地图。

优先只使用简单、可解释条件：

```text
near_obstacle == true
AND
behind_obstacle_free_extent > threshold
```

或：

```text
某扇区有效深度长期被近障碍截断
AND
局部 2D/2.5D 结构显示障碍后仍存在可通行空间
```

或：

```text
MAIN 因前方障碍提前停止
AND
远端外围区域未获得有效 RGB-D 视线
```

满足任一高置信条件：

```text
coverage_status = PERIMETER_BLIND
```

否则：

```text
coverage_status = SUFFICIENT
```

---

# 10. PARTIAL_PERIMETER：默认的外围补偿方式

如果存在明显外围盲区，不直接绕整个房间。

优先执行：

```text
PARTIAL_PERIMETER
```

目标：

> **用最短额外位移改变遮挡关系，只看原本看不到的区域。**

---

## 10.1 位置选择

根据盲区方向，在外围自由带中选择一个安全候选位置。

评分原则：

```text
VisibilityGain
-
MovementCost
-
RiskCost
```

即：

- 能让盲区露出来更多；
- 移动距离更短；
- 离墙和家具净空更好。

---

## 10.2 运动预算

建议：

```text
MAIN → PARTIAL_PERIMETER
0.5~0.8 m
```

原则上：

```text
< 1.0 m
```

---

## 10.3 不允许贴墙走

外围自由带只是用于获得新视角。

不执行真正 wall-following。

要求候选点满足：

```text
wall_clearance > safety_margin
furniture_clearance > safety_margin
```

---

## 10.4 扫描范围

只扫描原来的盲区。

禁止：

```text
第二次完整 360°
```

---

# 11. TARGET_VERIFY：已发现目标时的专项第二视角

如果 MAIN 或 INGRESS 已经发现新的红球候选，则允许：

```text
TARGET_VERIFY
```

其目的不是提高空间覆盖，而是：

- 第二视角确认；
- 改变遮挡关系；
- 获取更可靠深度；
- 排除红方块；
- 提高 Track 关联可靠性。

---

## 11.1 运动预算

建议：

```text
0.5~0.8 m
```

原则上：

```text
< 1.0 m
```

只围绕目标方向做局部观察。

---

# 12. TARGET_VERIFY 与 PARTIAL_PERIMETER 可以合并运动

如果：

```text
已有新红球
+
同时存在同方向外围盲区
```

不要分别去两个点。

优先选择一个：

```text
既能确认目标
+
又能改善盲区视野
```

的额外位置。

目标是：

```text
一次额外运动解决两个问题
```

---

# 13. EXTENDED_SWEEP：只有补偿后仍存在盲区才允许

执行 PARTIAL_PERIMETER 后重新检查：

```text
remaining_blind_zone?
```

如果：

```text
NO
→ EXIT
```

如果：

```text
YES
```

才允许升级：

```text
EXTENDED_SWEEP
```

---

## 13.1 EXTENDED_SWEEP 不等于固定完整绕圈

只沿外围安全自由带移动到：

> **剩余盲区能够被看到的位置。**

例如只有左后角没看见：

```text
MAIN
↓
LEFT PERIMETER
↓
LEFT-BACK
↓
完成观察
↓
RETURN / EXIT
```

不需要继续：

```text
绕后侧
→右侧
→门口
```

---

## 13.2 完整外围环绕只是最后 fallback

只有当：

```text
多个外围区域均持续不可见
+
外围自由带连续可通行
+
局部定位健康
+
运动安全
```

才允许接近完整环绕。

正常房间禁止直接进入该模式。

---

# 14. EGRESS_SCAN：退出过程中继续检测

从：

```text
MAIN / PERIMETER
→ Door
```

退出过程中：

```text
DangerDetector = ON
```

继续观察。

这一阶段对：

- 门口附近红球；
- MAIN 反方向目标；
- 进入时被家具挡住、退出时视角改变的目标；

尤其有价值。

---

# 15. EXIT 时发现新目标

如果 EGRESS_SCAN 首次发现新红球：

```text
NEW_TARGET_DURING_EXIT
```

则：

```text
STOP
↓
TARGET_VERIFY
↓
确认
↓
CONTINUE_EXIT
```

禁止为了任务状态已经进入 EXIT 就忽略新目标。

---

# 16. Observation Pose 与 Observation Opportunity 分离

最终不再把“观察”只理解成离散停靠点。

## 固定 Observation Pose

只有：

```text
MAIN_OBSERVATION
```

一个。

## Observation Opportunity

包括：

```text
INGRESS_SCAN
MAIN 360°
TARGET_VERIFY
PARTIAL_PERIMETER
EXTENDED_SWEEP
EGRESS_SCAN
```

这样可以：

> **不靠增加大量固定观察位，也能提高整个房间的视角覆盖。**

---

# 17. Room Frame 继续作为房间目标融合基准

ROOM_ANCHOR 建立：

```text
RoomAnchorFrame
```

所有：

```text
INGRESS
MAIN
TARGET_VERIFY
PARTIAL_PERIMETER
EXTENDED_SWEEP
EGRESS
```

发现的危险源优先记录在 Room Frame。

流程：

```text
Camera
↓
Base
↓
RoomAnchorFrame
↓
TargetHypothesis
```

退出后完成同门局部回环，再统一转换：

```text
P_world
```

降低短时 LIO 漂移导致的目标轨迹拆分。

---

# 18. 每房运动预算

为了防止外围补偿重新膨胀为完整房间探索：

```text
ROOM_ANCHOR         1
MAIN_OBSERVATION    1
TARGET_VERIFY       0~1
PARTIAL_PERIMETER   0~1
EXTENDED_SWEEP      0~1
```

并优先合并 TARGET_VERIFY 与外围补偿位置。

---

# 19. 明确禁止

Stage B 房间观察策略禁止：

1. 每个房间默认完整绕圈；
2. ROOM_ANCHOR 执行 360°；
3. MAIN 省略完整 360°；
4. MAIN 强制到达房间几何中心；
5. PARTIAL_PERIMETER 变成 wall-following；
6. 每个红球单独规划一个额外观察位；
7. 一次补偿后仍无限生成更多观察点；
8. EXTENDED_SWEEP 默认执行完整一圈；
9. 用危险源真值决定运行时路径；
10. 为了节省时间在明确存在盲区时直接 EXIT。

---

# 20. 专项日志

每个房间建议记录：

```text
room_id

anchor_pose
anchor_dwell_time

main_pose
anchor_to_main_distance

main_scan_complete
main_sector_valid_frames

perimeter_blind_detected
blind_zone_direction
blind_zone_count

target_verify_triggered

partial_perimeter_triggered
partial_perimeter_distance
partial_perimeter_visibility_gain

extended_sweep_triggered
extended_sweep_distance

danger_found_ingress
danger_found_main
danger_found_target_verify
danger_found_partial_perimeter
danger_found_extended_sweep
danger_found_egress

extra_motion_distance
room_total_motion_distance
room_observation_duration

local_loop_translation_correction
local_loop_rotation_correction
```

---

# 21. 如何判断外围补偿是否值得保留

重点统计：

```text
danger_found_main
vs
danger_found_perimeter
```

如果长期发现：

```text
外围补偿从未增加新危险源发现
```

则说明策略可能过度。

如果多次出现：

```text
MAIN 没看到
+
PARTIAL_PERIMETER / EGRESS 找到真实红球
```

则证明外围补偿有明确价值。

---

# 22. Stage B 观察专项验收指标

当前先不以任务时间为第一硬指标。

优先要求：

```text
完整访问房间情况下：
Danger Recall = 100%
False Positive = 0
Duplicate = 0
```

同时要求：

```text
正常房间不触发 EXTENDED_SWEEP
大多数房间只执行 MAIN
外围补偿有明确触发原因
额外移动有限
不存在无限绕行
```

---

# 23. 最终推荐状态机

```text
                    ENTER
                      │
                      ▼
                ROOM_ANCHOR
                只建立锚点
                      │
                      ▼
                INGRESS_SCAN
                      │
                      ▼
             MAIN_OBSERVATION
                      │
                  MAIN 360°
                      │
                      ▼
            PERIMETER_BLIND_CHECK
                 /              \
                /                \
          SUFFICIENT          BLIND
              │                  │
              │                  ▼
              │          PARTIAL_PERIMETER
              │                  │
              │         blind remains?
              │             /       \
              │           NO         YES
              │           │           │
              │           │     EXTENDED_SWEEP
              │           │           │
              └───────────┴───────────┘
                           │
                           ▼
                          EXIT
                           │
                      EGRESS_SCAN
                           │
                  new target during exit?
                      /          \
                    NO            YES
                    │              │
                    │       TARGET_VERIFY
                    │              │
                    └───────┬──────┘
                            ▼
                       LOCAL_LOOP
                            │
                            ▼
                         VISITED
```

---

# 24. 最终结论

Stage B 房间观察不采用“每房固定绕一圈”。

采用：

> **已有轨迹充分利用 + 一个 MAIN 360° + 遮挡驱动的渐进式外围补偿。**

优先级：

```text
INGRESS
↓
MAIN 360°
↓
PERIMETER_BLIND_CHECK
↓
必要时 PARTIAL_PERIMETER
↓
仍有盲区才 EXTENDED_SWEEP
↓
EGRESS
```

这样既能提高门口、靠墙自由带和家具背后危险源的可见概率，又避免每个房间固定执行长距离外围环绕。

最终原则：

> **不漏检优先，但额外移动必须由“真实盲区”驱动，而不是为了保险默认绕完整房间。**
