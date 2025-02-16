using RobotLibraryAlgorithm;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TrajectoryPositionList
{
    public class CartesianPositionList
    {
        public List<CartesianPosition> CartesianPositions;
        public int CpCount;
        public int CpTotalCount;
        public CartesianPositionList() 
        {
            CartesianPositions = new List<CartesianPosition>();
            CpCount = 0;
            CpTotalCount = 0;
        }

    }
}
