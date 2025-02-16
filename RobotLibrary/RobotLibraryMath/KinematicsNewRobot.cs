using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace RobotLibraryAlgorithm.KinematicsAlgorithm
{
    public class KinematicsNewRobot : Kinematics
    {
        public override CartesianPosition FkAngle(double[] angle)
        {
            
            return new CartesianPosition();
        }

        public override CartesianPosition FkAngle(double[] angles, Matrix4x4? Grip2Tool = null)
        {
            return new CartesianPosition();
        }

        

        public override CartesianPosition FkRad(double[] rad)
        {
            return new CartesianPosition();
        }

        public override Position Ik(CartesianPosition position)
        {
            return new Position();
        }

        public override void KinematicsInit()
        {
            
        }
    }
}
