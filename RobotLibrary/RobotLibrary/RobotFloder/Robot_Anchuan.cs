using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotLibrary
{
    public class Robot_Anchuan:Robot
    {
     
      public  Robot_Anchuan()
        {
            // 机器人参数 前三个参数是旋转轴xyz, 后三个参数是旋转中心坐标
            //用于界面模型的运动
            RobotSportsParams = new double[6, 6]
            {{0, 0, 1, 0, 0, 205 },
            {0, 1, 0, 40, 0, 330 },
            {0, -1, 0, 40, 0, 330+445 },
            {-1,0 , 0, 94, 0, 808 },
            {0, -1, 0, 480, 0, 808 },
            {-1, 0, 0, 480+80, 0, 808 }};
            //XYZABC
            RobotLimParams = new double[4, 6]
           {
               {-150,-85,-180,-180,-115,-360 },
               {150,117,60,180,115,360 },
               {-1005,-1005,0,-180,-180,-180 },
               {1005,1005,1402,180,180,180 }
           };
        }

        public override void MoveControlInit()
        {
           
        }
        public override void KinematicsInit()
        {
           
        }
        public override void Command(List<string> CommandStrings)
        {
            
        }
    }
}
