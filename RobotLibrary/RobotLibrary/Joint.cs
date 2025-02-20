using System;
using System.Drawing;
using System.Windows.Media.Media3D;

namespace RobotLibrary
{
    public class Joint:JointBase
    {
        /// <summary>
        /// �������� �������� ��������
        /// ֻ��Vector3D Axis ��ת��
        /// ֻ��Point3D Center ��ת���ĵ�
        /// double angle ��ת�Ƕȣ��Ƕ��ƣ�
        /// JointLimit jointlimit ��ת�Ƕȷ�Χ
        /// ���ڷ���ģ�͵��˶�
        /// </summary> 
        private Vector3D axis;
        public Vector3D Axis { get { return axis; } }
        private Point3D center;
        public Point3D Center { get {return center ; }  }
        public double angle ;
        public JointLimit jointlimit;
        /// <summary>
        /// �չ��캯����Joint��
        /// ö�ٱ����͡�Point3D�����Ĺ��캯����Joint��
        /// </summary>
        public Joint()
        {
            jointlimit = new JointLimit();
            model3D = new GeometryModel3D();
            modelvisual3D = new ModelVisual3D();
            jointpath = "";
        }
        /// <summary>
        /// ���캯����Joint��
        /// ��������ת��ö�ٱ�������ת���3D����
        /// ������ת�����ת���3D����
        /// </summary>
        /// <param name="eRobotRotAxis"></param>
        /// <param name="_center"></param>
        public Joint(ERobotRotAxis eRobotRotAxis, Point3D _center)
        {
            switch (eRobotRotAxis)
            {
                case ERobotRotAxis.RotX:
                    axis.X = 1;
                    axis.Y = 0;
                    axis.Z = 0;
                    break;
                case ERobotRotAxis.RotY:
                    axis.Y = 1;
                    axis.Z = 0;
                    axis.X = 0; 
                    break;
                case ERobotRotAxis.RotZ:
                    axis.Z = 1;
                    axis.X = 0;
                    axis.Y = 0;
                    break;
                default:
                    break;
            }
            center= _center;
            jointlimit = new JointLimit();
        }
        /// <summary>
        /// ������ת��
        /// </summary>
        /// <param name="_axis">��ת���������</param>
        public void SetRot(Vector3D _axis)
        {
            axis=_axis; 
        }
        /// <summary>
        /// ������ת��λ��
        /// </summary>
        /// <param name="Point3D">��ת���3D����</param>
        public void SetRotPoints(Point3D _center)
        {
            center = _center;
        }

    }
}