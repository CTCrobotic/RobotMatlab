classdef RobotLink
   properties
      clRobot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',1);   %robotic toolbox 工具集产生的类
      sRobotName   %
      sBaseStyle 
      nNumLink
      nNumJoint
      vLinkMother % tree 结构命名法
      vLinkChild
      vLinkSister
      mJointName
      mLinkName
   end

   properties (SetAccess = private)
       
      mJointStyle
   end
   
   methods
       function obj = RobotLink(RobotName,BaseStyle,NumLink,NumJoint)
           %obj.clRobot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',NumLink);
           obj.sRobotName = RobotName;
           obj.sBaseStyle = BaseStyle;
           obj.nNumLink = NumLink;
           obj.nNumJoint = NumJoint;
       end
       
       function obj = setLinkConnect(obj,LinkMother,LinkChild,LinkSister,LinkName,JointName)
           obj.vLinkMother = LinkMother;
           obj.vLinkSister = LinkSister;
           obj.vLinkChild = LinkChild;
           obj.mLinkName = LinkName;
           obj.mJoinrName = JointName;
       end
       
       function obj = setRobotInit(obj)
           Robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',obj.nNumLink);
           if(obj.nNumLink<=0)
               error('')
           end
           switch(obj.sBaseStyle) 
               case 'base'
                    body = robotics.RigidBody(obj.mLinkName{1});
                    joint = robotics.Joint(obj.mJointName{1}, 'revolute');
                    setFixedTransform(joint,trvec2tform([0 0 0]));
                    joint.JointAxis = [0 0 1];
                    body.Joint = joint;
                    addBody(robot, body, 'base');
               otherwise
           end
                   
       end
   end
end
