using RobotLibraryAlgorithm;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TrajectoryPositionList
{
    public class MovePositionList
    {
        public int MpCount;
        public List<Position> MovePositions;
        public MovePositionList() 
        {
            MovePositions = new List<Position>();
            MpCount = 0;
        }
    }
}
