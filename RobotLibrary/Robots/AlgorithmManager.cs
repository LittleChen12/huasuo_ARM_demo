using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Robots
{
    public class AlgorithmManager
    {
        public RobotLibraryAlgorithm.Algorithm algorithm;
        public RobotLibraryAlgorithm.InterPolation.MoveInterPolation MoveInterPolation;
        public RobotLibraryAlgorithm.InterPolation.TrajectoryPlanning Trajectory;
        public RobotLibraryAlgorithm.VirtualAxis.VirtualAxes VirtualAxes;
        //和机械臂相关：运动学算法
        public RobotLibraryAlgorithm.KinematicsAlgorithm.Kinematics kinematicsAlgorithm;
        public AlgorithmManager(string RobotType)
        {
            algorithm = new RobotLibraryAlgorithm.Algorithm();
            MoveInterPolation = new RobotLibraryAlgorithm.InterPolation.MoveInterPolation();
            Trajectory = new RobotLibraryAlgorithm.InterPolation.TrajectoryPlanning();
            VirtualAxes = new RobotLibraryAlgorithm.VirtualAxis.VirtualAxes();
            //初始化运动学算法
            kinematicsAlgorithm = KinematicsFctory.CreateKinematics(RobotType);
        }
    }
}
