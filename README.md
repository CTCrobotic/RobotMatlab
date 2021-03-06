# RobotMatlab
 使用Matlab代码解决机器人相关问题
 >  使用[CodeCogs](https://www.codecogs.com/latex/eqneditor.php)提供的公式编辑器生成公式图片！

## 说明
相关代码文件实现对应功能，用于理解相关概念


## No.1 SPD (stable PD)
稳定PD控制，使用平面二连杆演示了同样pd参数下的控制效果，正在编辑中

<a href="https://www.codecogs.com/eqnedit.php?latex={\tau&space;^k}&space;=&space;-&space;{k_p}({q^k}&space;-&space;q_{ref}^k)&space;-&space;{k_d}({\dot&space;q^k}&space;-&space;\dot&space;q_{ref}^k)\begin{array}{*{20}{c}}&space;{}&{(PD)}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?{\tau&space;^k}&space;=&space;-&space;{k_p}({q^k}&space;-&space;q_{ref}^k)&space;-&space;{k_d}({\dot&space;q^k}&space;-&space;\dot&space;q_{ref}^k)\begin{array}{*{20}{c}}&space;{}&{(PD)}&space;\end{array}" title="{\tau ^k} = - {k_p}({q^k} - q_{ref}^k) - {k_d}({\dot q^k} - \dot q_{ref}^k)\begin{array}{*{20}{c}} {}&{(PD)} \end{array}" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex={\tau&space;^k}&space;=&space;-&space;{k_p}({q^k}&space;&plus;&space;\Delta&space;t&space;\cdot&space;{\dot&space;q^k}&space;-&space;q_{ref}^{k&space;&plus;&space;1})&space;-&space;{k_d}({\dot&space;q^k}&space;&plus;&space;\Delta&space;t&space;\cdot&space;{\ddot&space;q^k})\begin{array}{*{20}{c}}&space;{}&{(SPD)}&space;\end{array}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?{\tau&space;^k}&space;=&space;-&space;{k_p}({q^k}&space;&plus;&space;\Delta&space;t&space;\cdot&space;{\dot&space;q^k}&space;-&space;q_{ref}^{k&space;&plus;&space;1})&space;-&space;{k_d}({\dot&space;q^k}&space;&plus;&space;\Delta&space;t&space;\cdot&space;{\ddot&space;q^k})\begin{array}{*{20}{c}}&space;{}&{(SPD)}&space;\end{array}" title="{\tau ^k} = - {k_p}({q^k} + \Delta t \cdot {\dot q^k} - q_{ref}^{k + 1}) - {k_d}({\dot q^k} + \Delta t \cdot {\ddot q^k})\begin{array}{*{20}{c}} {}&{(SPD)} \end{array}" /></a>


## No.2 RoboticToolBox
目的使基于matlab的robotic system toolbox库，建立更加便捷的搭建方式，初步设想是使用类实现，各连杆关系根据树结构搭建。已经实现二连杆的演示
