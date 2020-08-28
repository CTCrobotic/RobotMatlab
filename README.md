# RobotMatlab
 使用Matlab代码解决机器人相关问题


## 说明
相关代码文件实现对应功能，用于理解相关概念


## No.1 SPD (stable PD)
$({\tau ^k} =  - {k_p}({q^k} - q_{ref}^k) - {k_d}({\dot q^k} - \dot q_{ref}^k)\begin{array}{*{20}{c}}
{}&{(PD)}
\end{array})$
$({\tau ^k} =  - {k_p}({q^k} + \Delta t \cdot {\dot q^k} - q_{ref}^{k + 1}) - {k_d}({\dot q^k} + \Delta t \cdot {\ddot q^k})\begin{array}{*{20}{c}}
{}&{(SPD)}
\end{array})$
稳定PD控制，使用平面二连杆演示了同样pd参数下的控制效果，正在编辑中