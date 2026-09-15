你需要基于 PX4-Autopilot 开源代码实现固定翼时间参数化航迹跟踪前馈增强功能。

目标：

利用外部轨迹规划器输出的动力学参考量：

    n_n_ff      气流坐标系法向过载前馈
    n_t_ff      气流坐标系切向过载前馈
    p_ff        期望滚转角速度前馈

增强 PX4 固定翼控制性能。

注意：

该功能不是重新设计完整飞控，也不是替换 TECS、NPFG、姿态PID或角速度PID。

设计思想：

        u = u_ff + u_fb

其中：

u_ff:
    来自时间参数化轨迹和相位映射的动力学前馈

u_fb:
    保留PX4原有闭环控制，用于消除模型误差和扰动


==================================================
一、总体控制结构
==================================================

目标控制链：

trajectory phase mapping

        |
        |
        +----------------+
        |                |
       n_t_ff          n_n_ff
        |                |
        |                |
 energy feedforward   longitudinal feedforward
        |                |
        ↓                ↓

      TECS input     pitch-rate feedforward

        |
        |
        +----------------+

                p_ff

                |
                ↓

        roll-rate feedforward


最终：

PX4原有:

attitude controller
        |
        ↓
rate controller
        |
        ↓
actuator


保持不变。


==================================================
二、法向过载前馈 n_n_ff
==================================================


定义：

气流坐标系法向过载：

        n_n = L/(m*g)


其中：

        L = 0.5*rho*V^2*S*C_L


轨迹规划输出：

        n_n_ff


目标：

将法向过载需求转换为纵向姿态/角速度前馈。


采用短周期近似：

        n_n - 1 ≈ V/g * q


因此：

        q_ff = K_n * g/V * (n_n_ff - 1)


其中：

K_n 为新增参数。


要求：

增加参数：

        FW_NN_FF_GAIN


默认:

        0


范围:

        0~5


控制逻辑：

原PX4 pitch rate setpoint:

        q_cmd_original


修改为：

        q_cmd =
              q_cmd_original
              +
              q_ff


即：

        q_cmd =
              q_cmd_original
              +
              FW_NN_FF_GAIN*g/V*(n_n_ff-1)


注意：

该项仅作为feedforward。

禁止：

- 使用 n_n 反馈闭环
- 修改 rate controller
- 修改 elevator PID


==================================================
三、切向过载前馈 n_t_ff
==================================================


定义：

气流坐标系切向过载：

        n_t=(T-D)/(m*g)


动力学关系：

        n_t = V_dot/g + sin(gamma)


轨迹输出：

        n_t_ff


目标：

转换为速度/能量前馈。


实现方式：

优先采用：

        n_t_ff
        ↓
        V_dot_ff
        ↓
        V_ff


其中：

        V_dot_ff = g*(n_t_ff-sin(gamma))


积分：

        V_ff(t)=V0+integral(V_dot_ff)


将：

        V_ff

作为airspeed feedforward输入TECS。


不要替换TECS。

不要直接关闭TECS。


增加参数：

        FW_NT_FF_GAIN


控制：

        V_cmd =
              V_TECS
              +
              FW_NT_FF_GAIN*V_ff


==================================================
四、滚转角速度前馈 p_ff
==================================================


轨迹规划输出：

        p_ff


直接作用于：

vehicle_rates_setpoint.roll


PX4已有：

vehicle_rates_setpoint

其中：

roll/pitch/yaw

为body angular rate setpoint。


修改：

        p_cmd =
             p_cmd_original
             +
             p_ff_gain*p_ff


增加：

        FW_P_FF_GAIN


默认:

        0


范围:

        0~2


==================================================
五、代码修改原则
==================================================


禁止大规模重构。


优先方案：

新增一个feedforward模块：

例如：

src/modules/fw_ff_control/


负责：

1.
接收外部输入：

新增uORB topic：

vehicle_ff_setpoint


内容：

timestamp

float n_n_ff

float n_t_ff

float roll_rate_ff


2.
计算：

q_ff

p_ff

V_ff


3.
发布：

vehicle_rates_setpoint扩展量

或

传递给fw_att_control。


==================================================
六、需要新增的uORB消息
==================================================


新增：

msg/VehicleFeedforwardSetpoint.msg


内容：

uint64 timestamp

float32 normal_accel_ff

float32 tangential_accel_ff

float32 roll_rate_ff


单位：

normal_accel_ff:

n_n


tangential_accel_ff:

n_t


roll_rate_ff:

rad/s


==================================================
七、参数设计
==================================================


新增：

FW_NN_FF_GAIN

法向过载前馈增益


FW_NT_FF_GAIN

切向过载前馈增益


FW_P_FF_GAIN

滚转角速度前馈增益


要求：

加入yaml参数系统。


==================================================
八、日志
==================================================


增加Flight Review可查看量：

n_n_ff

q_ff

n_t_ff

V_ff

p_ff


同时记录：

vehicle_rates_setpoint

actuator_controls


方便验证前馈是否生效。


==================================================
九、验证要求
==================================================

完成以下自动验证：

1.
代码编译

要求：

make px4_sitl_default

通过。


2.
uORB检查

验证：

vehicle_ff_setpoint

正常发布和订阅。


3.
SITL测试

使用：

Gazebo fixed wing

或者

JSBSim fixed wing


验证：

输入恒定：

n_n_ff=1

结果：

q_ff≈0


输入：

n_n_ff>1


结果：

pitch rate setpoint增加。


输入：

p_ff

结果：

roll rate setpoint变化。


4.
日志验证：

检查：

q_ff

是否正确：

q_ff=K*g/V*(n_n_ff-1)


==================================================
十、必须明确哪些不能由AI自动完成
==================================================


以下必须人工完成：

1.
真实飞机气动参数确定

包括：

C_L_alpha

短周期参数

K_n


2.
SITL模型真实性验证


AI只能验证：

控制链是否工作。


不能证明：

真实飞机性能提升。


3.
HITL验证

需要：

Pixhawk硬件

真实传感器接口


4.
实际飞行测试


必须人工：

- 安全检查
- 小增益测试
- 增益逐步增加
- 检查发散风险


==================================================
最终目标
==================================================


实现：

相位映射轨迹

        ↓

(n_n_ff,n_t_ff,p_ff)

        ↓

PX4 feedforward augmentation

        ↓

TECS + attitude PID + rate PID

        ↓

actuator


实现时间航迹跟踪中的：

- 降低相位滞后
- 提高过载响应速度
- 保留PX4稳定反馈能力
