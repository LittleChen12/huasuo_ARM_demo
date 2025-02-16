using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace RobotLibraryAlgorithm
{
    public class Position
    {
        // 笛卡尔坐标（mm）和关节坐标(角度)
        private CartesianPosition pose;
        private JointPosition joints;

        public CartesianPosition Pose
        {
            get { return pose; }
            set { pose = value; }
        }

        public JointPosition Joints
        {
            get { return joints; }
            set { joints = value; }
        }
        public Position(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, Matrix4x4? Grip2Tool = null)
        {
            joints = new JointPosition(new double[] { joint1, joint2, joint3, joint4, joint5, joint6 });
            KinematicsAlgorithm.KinematicsHuaShu kinematicsAlgorithm = new KinematicsAlgorithm.KinematicsHuaShu();
            pose = new CartesianPosition(kinematicsAlgorithm.FkRad(new double[] { joint1, joint2, joint3, joint4, joint5, joint6 }, Grip2Tool));
        }

        public Position(double[] joint, Matrix4x4? Grip2Tool = null)
        {
            joints = new JointPosition(joint);
            KinematicsAlgorithm.KinematicsHuaShu kinematicsAlgorithm = new KinematicsAlgorithm.KinematicsHuaShu();
            pose = new CartesianPosition(kinematicsAlgorithm.FkRad(joint, Grip2Tool));
        }
        public Position(Position _temp)
        {
            pose = _temp.pose;
            joints = _temp.joints;
        }
        public Position(CartesianPosition _pose,JointPosition _joints)
        {
           
            pose = _pose;
            joints = _joints;
        }
        public Position()
        {
            joints = new JointPosition();
            pose = new CartesianPosition();
        }
    }
}
