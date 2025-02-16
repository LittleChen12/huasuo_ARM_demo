using rokae.clr;
using System;
using System.Collections.Generic;
using System.Threading;
using EventInfos = System.Collections.Generic.Dictionary<string, object>;

namespace xCoreSDK_CSharp
{
    public class RokaeRobotFunction
    {
        // 创建机器人对象并连接到六轴机器人
        private xMateRobot robot = new xMateRobot();

        private CancellationTokenSource cts = new CancellationTokenSource();

        /// <summary>
        /// 通过查询机器人的操作状态，等待机器人运动结束。
        /// </summary>
        private void waitRobot()
        {
            bool moving = true;
            while (moving)
            {
                Thread.Sleep(300);
                ErrorCode ec;
                OperationState st = robot.operationState(out ec);
                if (st == OperationState.idle || st == OperationState.unknown)
                {
                    moving = false;
                }
            }
        }

        /// <summary>
        /// 使机器人以当前末端姿态，在以当前末端位置为中心的平面内沿正方形路径运动一周。
        /// 平面的法向量方向等于当前末端的朝向。
        /// </summary>
        /// <param name="sideLength">正方形的边长，单位：米。</param>
        public void MoveInSquare(double sideLength)
        {
            ErrorCode ec;

            // 获取当前末端在参考坐标系中的位姿
            var currentPos = robot.cartPosture(CoordinateType.endInRef, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("获取当前位姿失败: " + ec.message);
                return;
            }

            double[] position = currentPos.trans; // [x, y, z]
            double[] rpy = currentPos.rpy;        // [roll, pitch, yaw]

            // 将RPY转换为旋转矩阵
            double[,] rotationMatrix = RPYToRotationMatrix(rpy[0], rpy[1], rpy[2]);

            // 获取末端的法向量（末端坐标系的Z轴）
            double[] normalVector = { rotationMatrix[0, 2], rotationMatrix[1, 2], rotationMatrix[2, 2] };

            // 获取末端坐标系的X轴和Y轴
            double[] xAxis = { rotationMatrix[0, 0], rotationMatrix[1, 0], rotationMatrix[2, 0] };
            double[] yAxis = { rotationMatrix[0, 1], rotationMatrix[1, 1], rotationMatrix[2, 1] };

            double halfSide = sideLength / 2;

            // 定义正方形四个角的偏移向量
            double[][] cornerOffsets = new double[4][];
            cornerOffsets[0] = AddVectors(ScaleVector(xAxis, halfSide), ScaleVector(yAxis, halfSide));    // 角点1 (+X, +Y)
            cornerOffsets[1] = AddVectors(ScaleVector(xAxis, -halfSide), ScaleVector(yAxis, halfSide));   // 角点2 (-X, +Y)
            cornerOffsets[2] = AddVectors(ScaleVector(xAxis, -halfSide), ScaleVector(yAxis, -halfSide));  // 角点3 (-X, -Y)
            cornerOffsets[3] = AddVectors(ScaleVector(xAxis, halfSide), ScaleVector(yAxis, -halfSide));   // 角点4 (+X, -Y)

            // 计算正方形四个角的绝对位置
            double[][] cornerPositions = new double[4][];
            for (int i = 0; i < 4; i++)
            {
                cornerPositions[i] = AddVectors(position, cornerOffsets[i]);
            }

            // 创建移动指令，依次移动到每个角点
            List<MoveCommand> cmds = new List<MoveCommand>();

            // 移动到第一个角点
            MoveCommand moveToFirstCorner = new MoveCommand
            {
                cartTarget = { trans = cornerPositions[0], rpy = rpy }
            };
            cmds.Add(moveToFirstCorner);

            // 移动到剩余的角点
            for (int i = 1; i < 4; i++)
            {
                MoveCommand moveCmd = new MoveCommand
                {
                    cartTarget = { trans = cornerPositions[i], rpy = rpy }
                };
                cmds.Add(moveCmd);
            }

            // 返回起始点，完成闭合
            MoveCommand moveBackToStart = new MoveCommand
            {
                cartTarget = { trans = cornerPositions[0], rpy = rpy }
            };
            cmds.Add(moveBackToStart);

            // 执行移动指令
            robot.executeCommand(MoveCommand.Type.MoveL, cmds, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("执行移动命令失败: " + ec.message);
                return;
            }

            // 等待运动完成
            waitRobot();
        }

        /// <summary>
        /// 将Roll、Pitch、Yaw角度转换为旋转矩阵。
        /// </summary>
        /// <param name="roll">Roll角，单位：弧度。</param>
        /// <param name="pitch">Pitch角，单位：弧度。</param>
        /// <param name="yaw">Yaw角，单位：弧度。</param>
        /// <returns>3x3的旋转矩阵。</returns>
        private double[,] RPYToRotationMatrix(double roll, double pitch, double yaw)
        {
            double c1 = Math.Cos(roll);
            double s1 = Math.Sin(roll);
            double c2 = Math.Cos(pitch);
            double s2 = Math.Sin(pitch);
            double c3 = Math.Cos(yaw);
            double s3 = Math.Sin(yaw);

            double[,] R = new double[3, 3];

            // 旋转矩阵按ZYX顺序计算：R = Rz(yaw) * Ry(pitch) * Rx(roll)
            R[0, 0] = c3 * c2;
            R[0, 1] = c3 * s2 * s1 - s3 * c1;
            R[0, 2] = c3 * s2 * c1 + s3 * s1;
            R[1, 0] = s3 * c2;
            R[1, 1] = s3 * s2 * s1 + c3 * c1;
            R[1, 2] = s3 * s2 * c1 - c3 * s1;
            R[2, 0] = -s2;
            R[2, 1] = c2 * s1;
            R[2, 2] = c2 * c1;

            return R;
        }

        /// <summary>
        /// 向量相加。
        /// </summary>
        /// <param name="v1">向量1。</param>
        /// <param name="v2">向量2。</param>
        /// <returns>相加后的向量。</returns>
        private double[] AddVectors(double[] v1, double[] v2)
        {
            return new double[] { v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2] };
        }

        /// <summary>
        /// 向量缩放。
        /// </summary>
        /// <param name="v">要缩放的向量。</param>
        /// <param name="scalar">缩放系数。</param>
        /// <returns>缩放后的向量。</returns>
        private double[] ScaleVector(double[] v, double scalar)
        {
            return new double[] { v[0] * scalar, v[1] * scalar, v[2] * scalar };
        }

        /// <summary>
        /// 将点从局部坐标系转换到全局坐标系。
        /// </summary>
        /// <param name="localPoint">局部坐标系中的点。</param>
        /// <param name="origin">局部坐标系在全局坐标系中的原点。</param>
        /// <param name="rotationMatrix">局部坐标系的旋转矩阵。</param>
        /// <returns>全局坐标系中的点。</returns>
        private double[] TransformPoint(double[] localPoint, double[] origin, double[,] rotationMatrix)
        {
            double[] globalPoint = new double[3];

            for (int i = 0; i < 3; i++)
            {
                globalPoint[i] = origin[i];
                for (int j = 0; j < 3; j++)
                {
                    globalPoint[i] += rotationMatrix[i, j] * localPoint[j];
                }
            }

            return globalPoint;
        }

        /// <summary>
        /// 获取末端执行器的位姿。
        /// </summary>
        /// <returns>包含末端位姿的数组。</returns>
        public double[] GetEndEffectorPose()
        {
            ErrorCode ec;
            double[] pose = robot.posture(CoordinateType.endInRef, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("获取末端位姿失败: " + ec.message);
                return null;
            }
            return pose;
        }

        /// <summary>
        /// 获取关节角度信息。
        /// </summary>
        /// <returns>包含关节角度的数组。</returns>
        public double[] GetJointAngles()
        {
            ErrorCode ec;
            double[] jointAngles = robot.jointPos(out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("获取关节角度失败: " + ec.message);
                return null;
            }
            return jointAngles;
        }

        /// <summary>
        /// 等待Jog运动停止。
        /// </summary>
        private void waitForJogStop()
        {
            bool moving = true;
            while (moving)
            {
                Thread.Sleep(300);
                ErrorCode ec;
                OperationState st = robot.operationState(out ec);
                if (st == OperationState.jog || st == OperationState.idle)
                {
                    moving = false;
                }
            }
        }

        ///<summary>
        ///重置机械臂连接状态。
        /// </summary>
        public void ResetPowerState()
        {
            disconnect();
        }

        /// <summary>
        /// 通过查询执行信息，等待运动结束。
        /// </summary>
        /// <param name="cmdID">指令ID。</param>
        /// <param name="index">最后一个目标点的索引。</param>
        private void waitForFinish(string cmdID, int index)
        {
            ErrorCode ec;
            while (true)
            {
                var info = robot.queryEventInfo(Event.moveExecution, out ec);
                var _cmdID = (string)info["cmdID"];
                var _idx = (int)info["wayPointIndex"];
                var _err = (ErrorCode)info["error"];
                if (_err.value != 0)
                {
                    Console.WriteLine($"指令 {_cmdID}:{_idx} 错误: {_err.message}");
                    return;
                }

                if (cmdID == _cmdID && _idx == index)
                {
                    Console.WriteLine($"指令 {cmdID}:{index} 已完成");
                    return;
                }

                Thread.Sleep(200);
            }
        }

        /// <summary>
        /// 打印运动执行信息。
        /// </summary>
        /// <param name="info">事件信息字典。</param>
        private void printMoveExecutionInfo(EventInfos info)
        {
            var _cmdID = (string)info["cmdID"];
            var _idx = (int)info["wayPointIndex"];
            var _err = (ErrorCode)info["error"];
            var _remark = (string)info["remark"];

            if (_err.value != 0)
            {
                Console.WriteLine($"ID: {_cmdID}:{_idx} ,错误: {_err.value}:{_err.message}");
            }
            if (!string.IsNullOrEmpty(_remark))
            {
                Console.WriteLine($"警告: {_remark}");
            }
            if ((bool)info["reachTarget"])
            {
                Console.WriteLine($"ID: {_cmdID}:{_idx} 达到目标点");
            }
        }

        /// <summary>
        /// 每2秒打印一次机器人的末端位姿、关节角度和关节速度。
        /// </summary>
        /// <param name="token">用于停止操作的取消令牌。</param>
        private void printState(CancellationToken token)
        {
            ErrorCode ec;
            do
            {
                Console.Write("末端在参考坐标系中的位姿: ");
                Array.ForEach(robot.posture(CoordinateType.endInRef, out ec), PrintHelper.print);

                var cartPos = robot.cartPosture(CoordinateType.flangeInBase, out ec);
                Console.Write("\n法兰在基坐标系中的位姿: ");
                Array.ForEach(cartPos.trans, PrintHelper.print);
                Array.ForEach(cartPos.rpy, PrintHelper.print);
                Console.Write("\n轴配置数据: ");
                foreach (var d in cartPos.confData)
                {
                    Console.Write($"{d} ");
                }

                Console.Write("\n关节位置 - [");
                Array.ForEach(robot.jointPos(out ec), PrintHelper.print);
                Console.Write("]\n关节速度 - [");
                Array.ForEach(robot.jointVel(out ec), PrintHelper.print);
                Console.Write("]\n关节力矩 - [");
                Array.ForEach(robot.jointTorque(out ec), PrintHelper.print);
                Console.WriteLine("]\n");
                Thread.Sleep(2000);

            } while (!token.IsCancellationRequested);
        }

        /// <summary>
        /// 准备机器人运动，切换到自动模式并上电。
        /// 可选地清除缓存并设置默认工具、速度和转弯区。
        /// </summary>
        /// <param name="default_speed">默认速度。</param>
        /// <param name="default_zone">默认转弯区。</param>
        public void Move_Preset(int default_speed, int default_zone)
        {
            ErrorCode ec;

            robot.setOperateMode(OperateMode.automatic, out ec);
            robot.setPowerState(true, out ec);

            robot.setEventWatcher(Event.moveExecution, printMoveExecutionInfo, out ec);

            robot.moveReset(out ec);
            robot.setDefaultSpeed(default_speed, out ec);
            robot.setDefaultZone(default_zone, out ec);

            var defaultToolset = new Toolset();
            robot.setToolset(defaultToolset, out ec);
        }

        /// <summary>
        /// 使用指定的IP地址连接到机器人。
        /// </summary>
        /// <param name="remoteIP">机器人的IP地址。</param>
        public void connect(string remoteIP)
        {
            robot.connectToRobot(remoteIP);
        }

        /// <summary>
        /// 与机器人断开连接。
        /// </summary>
        public void disconnect()
        {
            ErrorCode ec;
            robot.disconnectFromRobot(out ec);
            PrintHelper.checkError("已断开连接", ec);
        }

        /// <summary>
        /// 沿X轴移动机器人指定的距离。
        /// </summary>
        /// <param name="distance">移动距离，单位：米。</param>
        public void MoveForwardX(double distance)
        {
            MoveAlongAxis(distance, 0);
        }

        /// <summary>
        /// 沿Y轴移动机器人指定的距离。
        /// </summary>
        /// <param name="distance">移动距离，单位：米。</param>
        public void MoveForwardY(double distance)
        {
            MoveAlongAxis(distance, 1);
        }

        /// <summary>
        /// 沿Z轴移动机器人指定的距离。
        /// </summary>
        /// <param name="distance">移动距离，单位：米。</param>
        public void MoveForwardZ(double distance)
        {
            MoveAlongAxis(distance, 2);
        }

        /// <summary>
        /// 沿指定轴移动机器人指定的距离。
        /// </summary>
        /// <param name="distance">移动距离，单位：米。</param>
        /// <param name="axisIndex">轴索引（0为X轴，1为Y轴，2为Z轴）。</param>
        private void MoveAlongAxis(double distance, int axisIndex)
        {
            ErrorCode ec;
            var currentPos = robot.cartPosture(CoordinateType.endInRef, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("获取当前位姿失败: " + ec.message);
                return;
            }

            double[] newTrans = (double[])currentPos.trans.Clone();
            newTrans[axisIndex] += distance;

            MoveCommand moveCmd = new MoveCommand
            {
                cartTarget = { trans = newTrans, rpy = currentPos.rpy }
            };

            List<MoveCommand> cmds = new List<MoveCommand> { moveCmd };
            robot.executeCommand(MoveCommand.Type.MoveL, cmds, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("执行移动命令失败: " + ec.message);
                return;
            }

            waitRobot();
        }

        /// <summary>
        /// 将机器人移动到指定的位置。
        /// </summary>
        /// <param name="x">X坐标，单位：米。</param>
        /// <param name="y">Y坐标，单位：米。</param>
        /// <param name="z">Z坐标，单位：米。</param>
        public void MoveToPosition(double x, double y, double z)
        {
            ErrorCode ec;
            var currentPos = robot.cartPosture(CoordinateType.endInRef, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("获取当前位姿失败: " + ec.message);
                return;
            }

            MoveCommand moveCmd = new MoveCommand
            {
                cartTarget = { trans = new double[] { x, y, z }, rpy = currentPos.rpy }
            };

            List<MoveCommand> cmds = new List<MoveCommand> { moveCmd };
            robot.executeCommand(MoveCommand.Type.MoveL, cmds, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("执行移动命令失败: " + ec.message);
                return;
            }

            waitRobot();
        }

        /// <summary>
        /// 调整末端的Pitch角度。
        /// </summary>
        /// <param name="deltaPitch">Pitch角度的变化量，单位：弧度。</param>
        public void AdjustPitch(double deltaPitch)
        {
            AdjustRotation(1, deltaPitch);
        }

        /// <summary>
        /// 调整末端的Yaw角度。
        /// </summary>
        /// <param name="deltaYaw">Yaw角度的变化量，单位：弧度。</param>
        public void AdjustYaw(double deltaYaw)
        {
            AdjustRotation(2, deltaYaw);
        }

        /// <summary>
        /// 调整末端的Roll角度。
        /// </summary>
        /// <param name="deltaRoll">Roll角度的变化量，单位：弧度。</param>
        public void AdjustRoll(double deltaRoll)
        {
            AdjustRotation(0, deltaRoll);
        }

        /// <summary>
        /// 调整末端的旋转角度。
        /// </summary>
        /// <param name="index">旋转轴索引（0为Roll，1为Pitch，2为Yaw）。</param>
        /// <param name="deltaAngle">角度的变化量，单位：弧度。</param>
        private void AdjustRotation(int index, double deltaAngle)
        {
            ErrorCode ec;
            var currentPos = robot.cartPosture(CoordinateType.endInRef, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("获取当前位姿失败: " + ec.message);
                return;
            }

            double[] newRpy = (double[])currentPos.rpy.Clone();
            newRpy[index] += deltaAngle;

            MoveCommand moveCmd = new MoveCommand
            {
                cartTarget = { trans = currentPos.trans, rpy = newRpy }
            };

            List<MoveCommand> cmds = new List<MoveCommand> { moveCmd };
            robot.executeCommand(MoveCommand.Type.MoveL, cmds, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("执行移动命令失败: " + ec.message);
                return;
            }

            waitRobot();
        }

        /// <summary>
        /// 使机械臂在正上方环绕给定点，沿距离点一定距离的路径运动，
        /// 使机械臂末端始终指向该点，最大与Z轴的夹角为30度。
        /// </summary>
        /// <param name="x">目标点的X坐标，单位：米。</param>
        /// <param name="y">目标点的Y坐标，单位：米。</param>
        /// <param name="z">目标点的Z坐标，单位：米。</param>
        /// <param name="distance">规划机械臂运动的路径距离目标点的距离，单位：米。</param>
        public void CircleAroundPoint(double x, double y, double z, double distance)
        {
            ErrorCode ec;

            // 获取当前末端在参考坐标系中的位姿
            var currentPos = robot.cartPosture(CoordinateType.endInRef, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("获取当前位姿失败: " + ec.message);
                return;
            }

            // 设定扫查的分段数（可以调整，分段越多越平滑）
            int segments = 36;
            double angleIncrement = 2 * Math.PI / segments;

            // 创建移动指令列表
            List<MoveCommand> cmds = new List<MoveCommand>();

            for (int i = 0; i < segments; i++)
            {
                // 计算当前段的角度
                double angle = i * angleIncrement;

                // 计算目标位置，确保与Z轴的夹角不超过30度
                double targetX = x + distance * Math.Cos(angle);
                double targetY = y + distance * Math.Sin(angle);
                double targetZ = z + distance * Math.Tan(Math.PI / 6); // 确保最大与Z轴夹角为30度

                // 确保末端始终指向目标点
                double[] direction = { x - targetX, y - targetY, z - targetZ };
                double[] rpy = CalculateRPYFromDirection(direction);

                // 创建移动指令
                MoveCommand moveCmd = new MoveCommand
                {
                    cartTarget = { trans = new double[] { targetX, targetY, targetZ }, rpy = rpy }
                };
                cmds.Add(moveCmd);
            }

            // 执行移动指令
            robot.executeCommand(MoveCommand.Type.MoveL, cmds, out ec);
            if (ec.value != 0)
            {
                Console.WriteLine("执行移动命令失败: " + ec.message);
                return;
            }

            // 等待运动完成
            waitRobot();
        }

        /// <summary>
        /// 根据目标方向向量计算RPY角度，使机械臂末端始终指向目标点。
        /// </summary>
        /// <param name="direction">目标方向向量。</param>
        /// <returns>对应的RPY角度数组。</returns>
        private double[] CalculateRPYFromDirection(double[] direction)
        {
            // 计算末端姿态，使其朝向目标方向
            double yaw = Math.Atan2(direction[1], direction[0]);
            double pitch = -Math.Atan2(direction[2], Math.Sqrt(direction[0] * direction[0] + direction[1] * direction[1]));
            double roll = 0; // 保持Roll为0，确保末端水平

            return new double[] { roll, pitch, yaw };
        }
    }
}

