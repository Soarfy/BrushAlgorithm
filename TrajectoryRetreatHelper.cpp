#include "TrajectoryRetreatHelper.h"

void liftFromLastPointAndReturnToStart(DobotTcpDemo *demo,
                                       const Dobot::CDescartesPoint &lastPoint,
                                       const Dobot::CDescartesPoint &returnPoint,
                                       double liftMm)
{
    Dobot::CDescartesPoint lifted = lastPoint;
    lifted.z += liftMm;
    demo->moveRobotC(lifted, lifted);
    demo->moveRobotC(returnPoint, returnPoint);
}
