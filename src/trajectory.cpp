#include "auto_mapping_ros/trajectory.hpp"

using namespace std;

vector<float> Trajectory::GetWaypointDistances(const geometry_msgs::Pose &pose, bool inFront=true)
{
    geometry_msgs::TransformStamped world_to_car_msg_stamped = Transforms::WorldToCarTransform(pose);
    float carAngle = Transforms::GetCarOrientation(pose);

    // need to think about this
    float upperAngle = fmod(carAngle+M_PI/2,M_PI);
    float lowerAngle = fmod(carAngle-M_PI/2,M_PI);
    if (lowerAngle > upperAngle)
    {
        swap(lowerAngle, upperAngle);
    }
    vector<float> distances;
    for (auto itr = waypoints_.begin(); itr != waypoints_.end(); itr++)
    {
        pair<float, float> point = itr->GetPair();
        pair<float, float> transformedPoint = Transforms::TransformPoint(point, world_to_car_msg_stamped);
        float x = transformedPoint.first;
        float y = transformedPoint.second;
        if ((inFront && x>0) || (!inFront && x<0))
        {
            distances.push_back(sqrt(x*x+y*y));
        }
        else
        {
            distances.push_back(numeric_limits<float>::max());
        }
    }
    return distances;
}
Trajectory::Trajectory(float lookahead1, float lookahead2) : lookahead_1_(lookahead1), lookahead_2_(lookahead2)
{
}

Trajectory::~Trajectory()
{}

bool Trajectory::ReadCMAES(string filename)
{
    // string path = ros::package::getPath("milestone-3")+"/"+filename;
    string path = "/home/saumya/MPC_ws/src/F110-Final/"+filename;
    cout << path << endl;
    ifstream input(path);
    string coordX, coordY;
    vector<pair<float,float>> temp;
    if (input.is_open())
    {
        while (getline(input,coordX,','))
        {
            getline(input,coordY);
            temp.push_back(pair<float,float>(stof(coordX),stof(coordY)));
        }
    }
    else
    {
        cout << "Please run this from the root catkin_ws directory" << endl;
        return false;
    }
    for (unsigned int i = 0; i < temp.size(); i++)
    {
        float prev_x = temp[(i-1)%temp.size()].first;
        float prev_y = temp[(i-1)%temp.size()].second;
        float x = temp[i].first;
        float y = temp[i].second;
        float ori = atan2(y-prev_y,x-prev_x);
        State state;
        state.set_x(x);
        state.set_y(y);
        state.set_ori(ori);
        waypoints_.push_back(state);
    }
    return true;
}

int Trajectory::GetClosestIdx(const geometry_msgs::Pose pose, float lookahead)
{
    float minDistance = numeric_limits<float>::max();
    float argminDist = -1;
    vector<float> distances;
    if (lookahead < 0)
    {
        distances = GetWaypointDistances(pose, false);
    }
    else
    {
        distances = GetWaypointDistances(pose, true);
    }
    lookahead = abs(lookahead);
    for (unsigned int i = 0; i < distances.size(); i++)
    {
        float currentDistance = distances[i] - lookahead;
        if (currentDistance >= 0 && currentDistance < minDistance)  // && ori >= lowerAngle && ori <= upperAngle) {
        {
            argminDist = i;
            minDistance = currentDistance;
        }
    }
    return argminDist;
}

pair<pair<float,float>,int> Trajectory::FindClosest(pair<float,float> &globalPoint)
{
    pair<pair<float,float>,int> ans;
    pair<float,float> closest;
    int indexx;
    float min_dist = std::numeric_limits<float>::max();
    for (unsigned int i = 0; i < waypoints_.size(); i++)
    {
        float distance = Transforms::CalcDist(globalPoint,waypoints_[i].GetPair());
        if (distance<min_dist)
        {
            closest = waypoints_[i].GetPair();
            min_dist = distance;
            indexx = i;
        }
    }
    ans.first = closest;
    ans.second = indexx;
    return ans;
}

vector<pair<float,float>> Trajectory::GetPairPoints()
{
    vector<pair<float,float>> points;
    for (auto itr = waypoints_.begin(); itr != waypoints_.end(); itr++)
    {
        points.push_back(itr->GetPair());
    }
    return points;
}

bool Trajectory::IsPathCollisionFree(const geometry_msgs::Pose pose, OccGrid &occ_grid)
{
    int startingIdx = GetClosestIdx(pose, lookahead_1_);
    int endingIdx = GetClosestIdx(pose, lookahead_2_);
    // we're truly screwed of there's no point in front and behind us
    if (startingIdx == -1 && endingIdx == -1)
    {
        return false;
    }
    else if (startingIdx == -1)
    {
        startingIdx = (endingIdx-6)%waypoints_.size();
    }
    else if (endingIdx == -1)
    {
        endingIdx = (startingIdx+6)%waypoints_.size();
    }
    if (endingIdx < startingIdx)
    {
        endingIdx += waypoints_.size();
    }
    // std::cout << "Starting: " << startingIdx << "\tEnding: " << endingIdx << std::endl;
    for (int i = startingIdx; i < endingIdx; i++)
    {
        int idx1 = i%waypoints_.size();
        int idx2 = (i+1)%waypoints_.size();
        if (!occ_grid.CheckCollision(waypoints_[idx1].GetPair(),waypoints_[idx2].GetPair()))
        {
            return false;
        }
    }
    return true;
}
