using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Robots
{
    public class AlgorithmManager
    {
        /// <summary>
        /// algorithm：基础算法
        /// MoveInterPolation：插补算法
        /// Trajectory：轨迹规划算法
        /// VirtualAxes：虚拟轴算法
        /// kinematicsAlgorithm：运动学算法
        /// </summary>
        public RobotLibraryAlgorithm.Algorithm algorithm;
        public RobotLibraryAlgorithm.InterPolation.MoveInterPolation MoveInterPolation;
        public RobotLibraryAlgorithm.InterPolation.TrajectoryPlanning Trajectory;
        public RobotLibraryAlgorithm.VirtualAxis.VirtualAxes VirtualAxes;
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
