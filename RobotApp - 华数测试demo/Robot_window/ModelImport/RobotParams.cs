
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Shapes;

namespace Robot_window
{
    internal class RobotParams
    {
        //private string linkbasepath = "G:\RobotApp\RobotApp\Robot_window\Robot_window.csproj";
        //private string linkbasepath = "D:\\code\\RobotApp\\RobotApp\\Robot_window\\3D_Models\\";
        //private string linkbasepath = "D:\\研究生0\\机器人\\重庆机器人\\RobotApp - 华数测试demo\\Robot_window\\3D_Models";
        private string linkbasepath;
        public string LinkBasePath
		{
			get { return linkbasepath; }
			set { linkbasepath = value; }
		}

        internal double[,] robotparams = new double[6, 6]
        {{0, 0, 1, 0, 0, 256 },
        {0, 1, 0, 60, 0, 456 },
        {0, 1, 0, 60, 0, 896 },
        {1,0 , 0, 180.5, 0, 931 },
        {0, 1, 0, 470, 0, 931 },
        {1, 0, 0, 565, 0, 931 }};
        //络石
       // internal double[,] robotparams = new double[6, 6]
       //{{0, 0, 1, 0, 0, 296 },
       //   {0, 1, 0, 0, 0, 296 },
       //   {0, -1, 0, 0, 0, 296+490 },
       //   {0,0 , 1, 0, 0, 296+490+360 },
       //   {0, -1, 0, 0, -150, 296+490+360 },
       //   {0, 0, 1, 0, -150, 296+490+360+127 }};

        internal double[,] robotlimparams = new double[4, 6]
        {
           {-150,-85,-180,-180,-115,-360 },
           {150,117,60,180,115,360 },
           {-1005,-1005,0,-180,-180,-180 },
           {1005,1005,1402,180,180,180 }
        };



        //GP7参数
        //internal double[,] robotparams = new double[6, 6]
        //{{0, 0, 1, 0, 0, 205 },
        //{0, 1, 0, 40, 0, 330 },
        //{0, -1, 0, 40, 0, 330+445 },
        //{-1,0 , 0, 94, 0, 808 },
        //{0, -1, 0, 480, 0, 808 },
        //{-1, 0, 0, 480+80, 0, 808 }};


        /*
         * @name	  : RobotParamsRead
         * @brief	  : 读取机械臂各个关节轴旋转轴和旋转中心
         * @param	  : text
         * @return    :6*6的二维数组
         */
        public double[,] RobotParamsRead(string robotJointsParams) 
		{
            string[] content=null;
            try
            {
                 content= File.ReadAllLines(robotJointsParams);
            }
            catch (Exception ex)
            {
                Console.WriteLine("An error occurred: " + ex.Message);
            }
            for(int i=0;i< content.Length;i++)
            {
                string line = content[i];
                int[] num= line.Split(' ').Select(int.Parse).ToArray();
                for(int j=0;j<6;j++)
                {
                    robotparams[i,j]=num[j];
                }
            }
            return robotparams;
		}
        public double[,] RobotLimParamsRead() 
        {
            return robotlimparams;
        }


	}
}
