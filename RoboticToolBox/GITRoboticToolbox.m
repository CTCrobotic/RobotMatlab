close all
clear
clc

addpath('RobotClass');
%% 

%所有变量的选取都是在初始坐标系中得到的
RobotName = 'TL';
RobotBaseStyle = 'base';
LinkN = 3; JointN = 3;
LinkMother = [0 1 2];
LinkChild = [1 2 0];  %暂时还没有用到
LinkSister = [0 0 0];
LinkName = {'link1', 'link2', 'tool'};
JointName = {'joint1','joint2','fixed'};
JointStyle = [0 0 2]; % 0 1 2 分别表示旋转 移动 固定关节类型
JointAxis = [2 2 3]; % 0 1 2 分别表示x y z轴旋转 3 表示固定 无旋转
L = [0,0.2,0.3]; % 连杆的长度
JointRelatP = {[L(1) 0 0],[L(2) 0 0],[L(3) 0 0]}; % 在零位上，下一个关节在上一个关节坐标系的位置

%% 动力学参数
LinkMass = [1 1 1]; %单位 kg
LinkCoM = {[0 0 0],[0 0 0],[0 0 0]}; %质心 在
LinkInertia = {[1 1 1 0 0 0],[1 1 1 0 0 0],[1 1 1 0 0 0]};

%% 通过RobotLink类建立机器人模型
twoLinks = RobotLink(RobotName,RobotBaseStyle,LinkN,JointN);
twoLinks = twoLinks.setLinkConnect(LinkMother,LinkChild,LinkSister,LinkName,JointName,L);
twoLinks = twoLinks.setJointInform(JointAxis,JointRelatP,JointStyle);
twoLinks = twoLinks.setRobotInit(LinkMass,LinkCoM,LinkInertia);


%%  下面是依据自带的示例代码 计算逆运动学
% Show details of the robot to validate the input properties. The robot
% should have two non-fixed joints for the rigid bodies and a fixed body
% for the end-effector.
robot = twoLinks.clRobot;
showdetails(robot)

%% Define The Trajectory
% Define a circle to be traced over the course of 10 seconds. This circle
% is in the _xy_ plane with a radius of 0.15.
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

%% Inverse Kinematics Solution
% Use an |InverseKinematics| object to find a solution of robotic 
% configurations that achieve the given end-effector positions along the 
% trajectory. 

%%
% Pre-allocate configuration solutions as a matrix |qs|.
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);
%%
% Create the inverse kinematics solver. Because the _xy_ Cartesian points are the
% only important factors of the end-effector pose for this workflow, 
% specify a non-zero weight for the fourth and fifth elements of the 
% |weight| vector. All other elements are set to zero.
ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';

%%
% Loop through the trajectory of points to trace the circle. Call the |ik|
% object for each point to generate the joint configuration that achieves
% the end-effector position. Store the configurations to use later.

qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:);
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

%% Animate The Solution
% Plot the robot for each frame of the solution using that specific robot 
% configuration. Also, plot the desired trajectory.

%%
% Show the robot in the first configuration of the trajectory. Adjust the 
% plot to show the 2-D plane that circle is drawn on. Plot the desired 
% trajectory.
figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-0.1 0.7 -0.3 0.5])

%%
% Set up a <docid:robotics_ref.mw_9b7bd9b2-cebc-4848-a38a-2eb93d51da03 Rate> object to display the robot 
% trajectory at a fixed rate of 15 frames per second. Show the robot in
% each configuration from the inverse kinematic solver. Watch as the arm
% traces the circular trajectory shown.
framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end