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
      L
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
       
       function obj = setLinkConnect(obj,LinkMother,LinkChild,LinkSister,LinkName,JointName,L)
           obj.vLinkMother = LinkMother;
           obj.vLinkSister = LinkSister;
           obj.vLinkChild = LinkChild;
           obj.mLinkName = LinkName;
           obj.mJointName = JointName;
           obj.L = L;
       end
       
       function obj = setRobotInit(obj)
           Robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',obj.nNumLink);
           if(obj.nNumLink<=0)
               error('number of link cann`t be zero!')
           end
           switch(obj.sBaseStyle) 
               case 'base'
                    body = robotics.RigidBody(obj.mLinkName{1});
                    joint = robotics.Joint(obj.mJointName{1}, 'revolute');
                    setFixedTransform(joint,trvec2tform([obj.L(1) 0 0]));
                    joint.JointAxis = [0 0 1];
                    body.Joint = joint;
                    addBody(Robot, body, 'base');
               otherwise
           end
           for link_n = 2:obj.nNumLink
               body = robotics.RigidBody(obj.mLinkName{link_n});
                joint = robotics.Joint(obj.mJointName{link_n},'revolute');
                setFixedTransform(joint, trvec2tform([obj.L(link_n),0,0]));
                joint.JointAxis = [0 0 1];
                body.Joint = joint;
                addBody(Robot, body, obj.mLinkName{obj.vLinkMother(link_n)});
           end 
           obj.clRobot = Robot;
       end
   end
end
