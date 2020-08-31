close all
clear
clc

addpath('RobotClass');
%% 
%通过RobotLink类建立机器人模型
LinkN = 5; JointN = 5;
LinkMother = [0 1 2 1 4];
LinkChild = [1 2 0];
LinkSister = [0 0 0];
LinkName = {'link1', 'link2','link3', 'link4', 'tool'};
JointName = {'joint1','joint2','joint3','joint4','fixed'};
JointAxis = [2 2 2 2 2];
L = [0,0.3,0.3,0.1,0.5];
twoLinks = RobotLink('TL','base',LinkN,JointN);
twoLinks = twoLinks.setLinkConnect(LinkMother,LinkChild,LinkSister,LinkName,JointName,L);
twoLinks = twoLinks.setJointAxis(JointAxis);
twoLinks = twoLinks.setRobotInit();
%%

robot = twoLinks.clRobot;
showdetails(robot)
view(2)
show(robot,[0 0 0 pi/2 pi/2]');
drawnow