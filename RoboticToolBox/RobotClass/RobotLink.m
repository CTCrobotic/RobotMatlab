classdef RobotLink
   properties
      clRobot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',1);   %robotic toolbox 工具集产生的类 输入的数据应该是列向量
      sRobotName   % 机器人的名称
      sBaseStyle % 基座的类型
      nNumLink % 连杆的数量
      nNumJoint % 关节的数量
      vLinkMother % tree 结构命名法
      vLinkChild 
      vLinkSister
      mJointName % 关节名称
      mLinkName % 连杆名称
      vLinkLength % 连杆的长度
   end

   properties (SetAccess = private)
      mJointStyle = [];% 每个关节的类型 默认是旋转
      mJointAxis = [];  % 关节转轴的方向
      mJointRelatPos = []; % 包括第一个关节相对于世界坐标系的位置 所以数量应该等于关节数量
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
           obj.vLinkLength = L;
       end
       
       function obj = setJointInform(obj,JointAxis,JointRelatPos,JointStyle)
           if(~(obj.nNumJoint == size(JointAxis,2)))
               error('JointAxis number don`t equit nNumJoint!')
           end
           for JA_k = 1:size(JointAxis,2)
               switch (JointAxis(JA_k))
                   case 0
                       obj.mJointAxis{JA_k}= [1 0 0];
                   case 1
                       obj.mJointAxis{JA_k}= [0 1 0];
                   case 2
                       obj.mJointAxis{JA_k}= [0 0 1];
                   case 3
                       obj.mJointAxis{JA_k}= nan;
                   otherwise
               end
           end
           if(~(obj.nNumJoint == size(JointRelatPos,2)))
               error('JointRelatPos number don`t equit nNumJoint!')
           end
           obj.mJointRelatPos = JointRelatPos;
           if(~(obj.nNumJoint == size(JointStyle,2)))
               error('JointStyle number don`t equit nNumJoint!')
           end
           for JS_k = 1:size(JointStyle,2)
               switch (JointStyle(JS_k))
                   case 0
                       obj.mJointStyle{JS_k}= 'revolute';
                   case 1
                       obj.mJointStyle{JS_k}= 'prismatic';
                   case 2
                       obj.mJointStyle{JS_k}= 'fixed';
                   otherwise
               end
           end
       end
       
       function obj = setRobotInit(obj,LinkMass,LinkCoMass,LinkInertia)
           Robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',obj.nNumLink);
           if(obj.nNumLink<=0)
               error('number of link cann`t be zero!')
           end
           for link_n = 1:obj.nNumLink
               if(obj.vLinkMother(link_n) == 0)
                   switch(obj.sBaseStyle) 
                        case 'base'
                            body = robotics.RigidBody(obj.mLinkName{link_n});
                            body.Mass = LinkMass(link_n);
                            body.CenterOfMass = LinkCoMass{link_n};
                            body.Inertia = LinkInertia{link_n};
                            body.addVisual('Mesh','link5.stl')
                            joint = robotics.Joint(obj.mJointName{link_n}, obj.mJointStyle{link_n});
                            setFixedTransform(joint,trvec2tform(obj.mJointRelatPos{link_n}));
                            if(~isnan(obj.mJointAxis{link_n}))
                                joint.JointAxis = obj.mJointAxis{link_n};
                            end
                            body.Joint = joint;
                            addBody(Robot, body, 'base');
                       otherwise
                   end
               else
                   body = robotics.RigidBody(obj.mLinkName{link_n});
                   body.Mass = LinkMass(link_n);
                   body.addVisual('Mesh','link5.stl')
                   body.CenterOfMass = LinkCoMass{link_n};
                   body.Inertia = LinkInertia{link_n};
                   joint = robotics.Joint(obj.mJointName{link_n}, obj.mJointStyle{link_n});
                   setFixedTransform(joint, trvec2tform(obj.mJointRelatPos{link_n}));
                   if(~isnan(obj.mJointAxis{link_n}))
                        joint.JointAxis = obj.mJointAxis{link_n};
                   end
                   body.Joint = joint;
                   addBody(Robot, body, obj.mLinkName{obj.vLinkMother(link_n)});
               end
           end
           Robot.Gravity = [0 0 -9.81];
           obj.clRobot = Robot;
       end
   end
end
