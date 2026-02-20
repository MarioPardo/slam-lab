#include "PoseGraph.h"
#include "icp_matcher.h"

namespace slam{

bool PoseGraph::shouldAddKeyframe(const Pose2D& pose)
{
    if(nodes.empty())
        return true;

    Pose2D prevPose = nodes.back().pose;

    float dx = pose.x - prevPose.x;
    float dy = pose.y - prevPose.y;
    float dist = sqrt(dx*dx + dy*dy);

    float dtheta = pose.theta - prevPose.theta;
    if(dtheta > M_PI) dtheta -= 2*M_PI;
    if(dtheta < -M_PI) dtheta += 2*M_PI;

    return (dist > distThreshold || abs(dtheta) > angleThreshold);  
}

bool PoseGraph:: tryAddKeyframe(const Pose2D& pose, const LidarScan& scan, double timestamp)
{
    if(!shouldAddKeyframe(pose))
        return false;

    Node newNode = {nodeID, timestamp, pose, scan};
    nodeID++;

    if(!nodes.empty())
    {
        Transform2D rel_trans = slam::computePoseDelta(nodes.back().pose, pose);
        Edge newEdge = {nodes.back().id, newNode.id, rel_trans};
        edges.push_back(newEdge);
    }

    nodes.push_back(newNode);
    return true;
}

}