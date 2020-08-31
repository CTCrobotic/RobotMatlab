classdef RobotLink
   properties
      clRobot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',1);   %robotic toolbox ���߼���������
      sRobotName   % �����˵�����
      sBaseStyle % ����������
      nNumLink % ���˵�����
      nNumJoint % �ؽڵ�����
      vLinkMother % tree �ṹ������
      vLinkChild 
      vLinkSister
      mJointName % �ؽ�����
      mLinkName % ��������
      L
   end

   properties (SetAccess = private)
      mJointStyle = [];% ÿ���ؽڵ����� Ĭ������ת
      mJointAxis = [];  % �ؽ�ת��ķ���
      mJointRelatPos = []; % ������һ���ؽ��������������ϵ��λ�� ��������Ӧ�õ��ڹؽ�����
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
       
       function obj = setJointAxis(obj,JointAxis)
           if(~(obj.nNumJoint == size(JointAxis,2)))
               error('JointStyle number don`t equit nNumJoint!')
           end
           for JS_k = 1:size(JointAxis,2)
               switch (JointAxis(JS_k))
                   case 0
                       obj.mJointAxis{JS_k}= [1 0 0];
                   case 1
                       obj.mJointAxis{JS_k}= [0 1 0];
                   case 2
                       obj.mJointAxis{JS_k}= [0 0 1];
                   otherwise
               end
           end
       end
       
       function obj = setJointRelatPos(obj,JointRelatPos)
           if(~(obj.nNumJoint == size(JointRelatPos,2)))
               error('JointRelatPos number don`t equit nNumJoint!')
           end
           obj.mJointRelatPos = JointRelatPos;
       end
       
       function obj = setRobotInit(obj)
           Robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',obj.nNumLink);
           if(obj.nNumLink<=0)
               error('number of link cann`t be zero!')
           end
           for link_n = 1:obj.nNumLink
               if(obj.vLinkMother(link_n) == 0)
                   switch(obj.sBaseStyle) 
                        case 'base'
                            body = robotics.RigidBody(obj.mLinkName{link_n});
                            joint = robotics.Joint(obj.mJointName{link_n}, 'revolute');
                            setFixedTransform(joint,trvec2tform([obj.L(link_n) 0 0]));
                            joint.JointAxis = obj.mJointAxis{link_n};
                            body.Joint = joint;
                            addBody(Robot, body, 'base');
                       otherwise
                   end
               else
                   body = robotics.RigidBody(obj.mLinkName{link_n});
                   joint = robotics.Joint(obj.mJointName{link_n},'revolute');
                   setFixedTransform(joint, trvec2tform([obj.L(link_n),0,0]));
                   joint.JointAxis = obj.mJointAxis{link_n};
                   body.Joint = joint;
                   addBody(Robot, body, obj.mLinkName{obj.vLinkMother(link_n)});
               end
           end 
           obj.clRobot = Robot;
       end
       
       function obj = setRobotDynamic(obj)
           
       end
   end
end
