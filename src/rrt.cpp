#include "auto_mapping_ros/rrt.hpp"

RRT::~RRT()
{
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh, OccGrid occ_grid): nh_(nh), occ_grid_(occ_grid), gen((std::random_device())())
{

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    bool DEBUG = true;
    string rrt_node = "/rrt_node";
    if (DEBUG)
    {
        rrt_node = "";
    }

    nh_.getParam(rrt_node+"/max_goal_distance", max_goal_distance);
    nh_.getParam(rrt_node+"/min_goal_distance", min_goal_distance);
    nh_.getParam(rrt_node+"/goal_epsilon", goal_epsilon);
    nh_.getParam(rrt_node+"/max_expansion_dist", max_expansion_dist);
    nh_.getParam(rrt_node+"/rrt_iters", rrt_iters);
    nh_.getParam(rrt_node+"/angle_limit",angle_limit);
    nh_.getParam(rrt_node+"/steer_angle",steer_angle);
    nh_.getParam(rrt_node+"/rrt_radius_sq",rrt_radius_sq);
    nh_.getParam(rrt_node+"/shortcut_iters",shortcut_iters);

    // FIXME: we only want the grid in front of car right?
    int divide = 3;
    x_dist = uniform_real_distribution<double>(0.4,1.5 * max_goal_distance);
    y_dist = uniform_real_distribution<double>(-max_goal_distance,max_goal_distance);

    goal_pub = nh_.advertise<visualization_msgs::Marker>("goal_marker", 1);
    line_pub = nh_.advertise<visualization_msgs::Marker>("path_marker", 1);
    tree_pub = nh_.advertise<visualization_msgs::Marker>("tree_marker", 1);
    shortcut_pub = nh_.advertise<visualization_msgs::Marker>("shortcut_marker", 1);

    // TODO: create a occupancy grid
    ROS_INFO("Created new RRT Object.");
}

float RRT::get_distance(pair<float,float> p1, pair<float, float> p2)
{
    return sqrt(pow(p1.first-p2.first,2)+pow(p1.second-p2.second,2));
}

float RRT::getPathLength(vector<pair<float,float>> &path)
{
    float length = 0;
    if (path.size()>1)
    {
        for (int i = 1; i < path.size(); i++)
        {
            length += get_distance(path[i-1], path[i]);
        }
    }
    return length;
}

// assume car always starts at beginning of path
vector<State> RRT::getRRTStates(float dt=0.01, int horizon=50)
{
    vector<State> states;
    int vectorSize = rrtPath.size();
    float velocity = 3;
    float pathLength = getPathLength(rrtPath);
    float pathTime = pathLength/velocity;
    float deltaT = dt*horizon;
    int steps = pathTime/dt;
    if (vectorSize > 3)
    {
        vector<pair<float, float>> stepsPath = smooth_path(rrtPath, steps+2);
        if (stepsPath.size() >= horizon)
        {
            for (int i = 1; i < horizon+1; i++)
            {
                float prev_x = stepsPath[i-1].first;
                float prev_y = stepsPath[i-1].second;

                float x = stepsPath[i].first;
                float y = stepsPath[i].second;
                float ori = atan2(y-prev_y,x-prev_x);
                State state;
                state.set_x(x);
                state.set_y(y);
                state.set_ori(ori);
                states.push_back(state);
            }
        }
        else
        {
            State lastState;
            for (int i = 1; i < stepsPath.size(); i++)
            {
                float prev_x = stepsPath[i-1].first;
                float prev_y = stepsPath[i-1].second;

                float x = stepsPath[i].first;
                float y = stepsPath[i].second;
                float ori = atan2(y-prev_y,x-prev_x);
                State state;
                state.set_x(x);
                state.set_y(y);
                state.set_ori(ori);
                lastState = state;
                states.push_back(state);
            }
            for (int i = stepsPath.size(); i < horizon; i++)
            {
                states.push_back(lastState);
            }
        }
    }
    return states;
}

vector<pair<float,float>> RRT::shortcutPath(vector<pair<float,float>> path)
{
    if (path.size()>2)
    {
        random_device device;
        mt19937 generator(device());
        uniform_real_distribution<double> t_(0,1);
        for (int i = 0; i < shortcut_iters; i++)
        {
            if (path.size()<3)
            {
                break;
            }
            uniform_int_distribution<int> distribution(0,path.size()-2);
            int startIdx;
            int endIdx;
            do
            {
                startIdx = distribution(generator);
                endIdx = distribution(generator);
            }
            while (startIdx==endIdx);
            float length = getPathLength(path);
            if (startIdx > endIdx)
            {
                swap(startIdx, endIdx);
            }
            float start_t = t_(generator);
            float end_t = t_(generator);

            pair<float,float> start_shortcut;
            pair<float,float> end_shortcut;

            start_shortcut.first = (1-start_t)*path[startIdx].first +start_t*path[startIdx+1].first;
            start_shortcut.second = (1-start_t)*path[startIdx].second+start_t*path[startIdx+1].second;

            end_shortcut.first = (1-end_t)*path[endIdx].first +end_t*path[endIdx+1].first;
            end_shortcut.second = (1-end_t)*path[endIdx].second+end_t*path[endIdx+1].second;

            vector<pair<float,float>> tempPath(path.begin(),path.begin()+startIdx+1);
            tempPath.push_back(start_shortcut);
            tempPath.push_back(end_shortcut);
            tempPath.insert(tempPath.end(), path.begin()+endIdx+1, path.end());

            if (isPathCollisionFree(tempPath))
            {
                if (getPathLength(tempPath) < getPathLength(path))
                {
                    path = tempPath;
                }
            }
        }
    }
    return path;
}

vector<pair<float,float>> RRT::smooth_path(vector<pair<float,float>> path, int discrete=100)
{
    if (path.size()>3)
    {
        vector<double> x, y;
        for (auto itr = path.begin(); itr != path.end(); itr++)
        {
            x.push_back(itr->first);
            y.push_back(itr->second);
        }
        xt::xarray<double> t = xt::linspace<double>(0,1,x.size());
        xt::xarray<double> x_ = xt::adapt(x);
        xt::xarray<double> y_ = xt::adapt(y);
        auto tck1 = xt::interpolate::splrep(t, x_, 3, 1);
        auto tck2 = xt::interpolate::splrep(t, y_, 3, 1);

        xt::xarray<double> t_evaluate = xt::linspace<double>(0,1,discrete);

        xt::xarray<double> xs = xt::interpolate::splev(t_evaluate, tck1);
        xt::xarray<double> ys = xt::interpolate::splev(t_evaluate, tck2);

        vector<double> x_vec(xs.begin(), xs.end()-1);
        vector<double> y_vec(ys.begin(), ys.end()-1);
        vector<pair<float,float>> convertedPath;
        for (int i = 0; i < x_vec.size(); i++)
        {
            convertedPath.push_back(make_pair<float,float>(x_vec[i],y_vec[i]));
        }
        return convertedPath;
    }
    return path;
}

void RRT::updateRRT(geometry_msgs::Pose &pose_update, OccGrid& occ_grid, std::pair<float, float> carFrameWaypoint, std::pair<float, float> globalFrameWaypoint)
{
    current_pose_ = pose_update;
    occ_grid_ = occ_grid;
    int index = build_tree(carFrameWaypoint, globalFrameWaypoint);
    vector<pair<float,float>> shortPath, prePath;
    if (index != -1)
    {
        prePath = smooth_path(find_path(tree, tree[index]));
        rrtPath = smooth_path(shortcutPath(prePath));
        // cout << "RRRRT SIZE " << rrtPath.size();
    }
    visualization_msgs::MarkerArray rrt_marker = gen_RRT_markers();
    visualization_msgs::Marker path_marker = gen_path_marker(prePath);
    visualization_msgs::Marker shortcut_marker = gen_path_marker(rrtPath,0.5,0,0.5);
    visualization_msgs::Marker tree_marker = gen_tree_marker(tree);

    tree_pub.publish(tree_marker);
    line_pub.publish(path_marker);
    shortcut_pub.publish(shortcut_marker);
}


// returns point closest to waypoint
int RRT::build_tree(std::pair<float, float> targetWaypoint, std::pair<float, float> targetWaypointGlobalCoords)
{
    float current_angle = atan2(2 * current_pose_.orientation.w * current_pose_.orientation.z, 1 - 2 * current_pose_.orientation.z * current_pose_.orientation.z);
    tree.clear();

    float posex = current_pose_.position.x;
    float posey = current_pose_.position.y;
    Node root = Node(posex, posey, true);
    root.parent = -1;

    posex = current_pose_.position.x + 0.275 * cos(current_angle);
    posey = current_pose_.position.y + 0.275 * sin(current_angle);
    Node root_plus = Node(posex, posey, true);
    root_plus.parent = 0;
    tree.push_back(root);
    tree.push_back(root_plus);

    publish_marker(targetWaypointGlobalCoords.first, targetWaypointGlobalCoords.second);

    for (int i = 0; i < rrt_iters; i++)
    {
        // FIXME: i think this should be from 0,0
        vector<double> x_rand;
        Node x_new;
        int x_nearest_idx;
        // need to check distance here
        x_rand.push_back(targetWaypointGlobalCoords.first);
        x_rand.push_back(targetWaypointGlobalCoords.second);
        x_nearest_idx = nearest(tree, x_rand);
        x_new.x = x_rand[0];
        x_new.y = x_rand[1];
        x_new.parent = x_nearest_idx;
        vector<double> tmp;
        tmp.push_back(targetWaypointGlobalCoords.first);
        tmp.push_back(targetWaypointGlobalCoords.second);
        float angle = angle_cost(tree[x_nearest_idx], tmp);
        // if (check_collision(tree[x_nearest_idx], x_new) && abs(angle)<steer_angle)
        // {
        //     tree.push_back(x_new);
        //     // ROS_INFO("breaking out coz of shortcircuit %f", angle);
        //     break;
        // } else
        {
            x_rand = sample();
            x_nearest_idx = nearest(tree, x_rand);
            x_new =  steer(tree[x_nearest_idx], x_rand);
            if (check_collision(tree[x_nearest_idx], x_new) && angle_cost(tree, tree[x_nearest_idx],x_new)<angle_limit)
            {
                std::vector<int> nearby = near(tree, x_new);

                int x_min = x_nearest_idx;
                double c_min = cost(tree, tree[x_nearest_idx]) + line_cost(tree[x_nearest_idx], x_new);
                for (int current_near = 0; current_near < nearby.size(); ++current_near)
                {
                    if (check_collision(tree[nearby[current_near]], x_new) && (cost(tree, tree[nearby[current_near]]) + line_cost(x_new, tree[nearby[current_near]])<c_min))
                    {
                        x_min = nearby[current_near];
                        c_min = cost(tree, tree[nearby[current_near]]) + line_cost(x_new, tree[nearby[current_near]]);
                    }
                }
                x_new.parent = x_min;
                tree.push_back(x_new);
                for (int current_near = 0; current_near < nearby.size(); ++current_near)
                {
                    if (check_collision(tree[nearby[current_near]], x_new) && (cost(tree, x_new) + line_cost(x_new, tree[nearby[current_near]])) < cost(tree, tree[nearby[current_near]]))
                    {
                        tree[nearby[current_near]].parent = tree.size()-1;
                    }
                }
                if (is_goal(x_new, targetWaypointGlobalCoords.first, targetWaypointGlobalCoords.second))
                {
                    // ROS_INFO("breaking out coz GOAL FOUND");
                    // break;
                }
            }
        }
    }

    float minDistance = numeric_limits<float>::max();
    int minIndex = -1;
    for (auto itr = tree.begin(); itr != tree.end(); itr++)
    {
        pair<float,float> point(itr->x,itr->y);
        geometry_msgs::TransformStamped transform_msg = Transforms::WorldToCarTransform(current_pose_);
        std::pair<float, float> transformedPoint = Transforms::TransformPoint(point, transform_msg);
        if (transformedPoint.first > 0)
        {
            float distance = get_distance(transformedPoint, targetWaypoint);
            if (distance < minDistance)
            {
                minDistance = distance;
                minIndex = itr-tree.begin();
            }
        }
    }
    return minIndex;
}

std::vector<double> RRT::sample()
{
    std::vector<double> sampled_point;
    float x = x_dist(gen);
    float y = y_dist(gen);
    pair<float,float> newPoint = Transforms::CarPointToWorldPoint(x,y,current_pose_);
    sampled_point.push_back(newPoint.first);
    sampled_point.push_back(newPoint.second);
    return sampled_point;
}


int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point)
{
    float min_dist = 100;
    float curr_dist;
    int nearest_node = 0;
    for (int i = 0; i<tree.size(); i++)
    {
        curr_dist = pow((tree[i]).x-sampled_point[0],2) + pow((tree[i]).y-sampled_point[1],2);
        if (curr_dist<min_dist)
        {
            min_dist = curr_dist;
            nearest_node = i;
        }
    }
    // TODO: linearly traverse all the nodes in the treegrid_size

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point)
{
    double curr_distance = sqrt(pow(nearest_node.x-sampled_point[0],2) + pow(nearest_node.y-sampled_point[1],2));
    Node new_node;
    vector<double> new_point;
    if (curr_distance > max_expansion_dist)
    {
        new_point.push_back(nearest_node.x + ((sampled_point[0]-nearest_node.x)*max_expansion_dist/curr_distance));
        new_point.push_back(nearest_node.y + ((sampled_point[1]-nearest_node.y)*max_expansion_dist/curr_distance));
    }
    else
    {
        new_point.push_back(sampled_point[0]);
        new_point.push_back(sampled_point[1]);
    }
    float angle = angle_cost(nearest_node, new_point);
    if (abs(angle)>steer_angle && nearest_node.is_root == false)
    {
        double distance = min(curr_distance, max_expansion_dist);
        Node parent = tree[nearest_node.parent];
        double prev_angle = atan2(nearest_node.y-parent.y, nearest_node.x-parent.x);
        float steer = (angle>0) ? steer_angle : -steer_angle;
        new_point[0] = nearest_node.x+distance*cos(steer+prev_angle);
        new_point[1] = nearest_node.y+distance*sin(steer+prev_angle);
    }

    new_node.x = new_point[0];
    new_node.y = new_point[1];

    return new_node;
}

bool RRT::check_collision(pair<float,float> a, pair<float,float> b)
{
    Node n1 = Node(a.first, a.second, false);
    Node n2 = Node(b.first, b.second, false);
    return check_collision(n1, n2);
}

bool RRT::check_collision(Node a, Node b)
{
    return occ_grid_.CheckCollision(a.x,a.y,b.x,b.y);
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y)
{
    bool close_enough = false;
    float curr_dist = sqrt(pow(latest_added_node.x-goal_x,2) + pow(latest_added_node.y-goal_y,2));
    if (curr_dist<goal_epsilon)
    {
        close_enough = true;
    }
    return close_enough;
}

std::vector<pair<float,float>> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node)
{
    bool root = false;
    std::vector<pair<float,float>> found_path;
    Node curr_node = latest_added_node;
    pair<float,float> nodexy;
    nodexy.first = (float)curr_node.x;
    nodexy.second= (float)curr_node.y;
    found_path.push_back(nodexy);
    while (!curr_node.is_root)
    {
        curr_node = tree[curr_node.parent];
        root = curr_node.is_root;
        nodexy.first = curr_node.x;
        nodexy.second = curr_node.y;
        found_path.push_back(nodexy);
    }
    std::reverse(found_path.begin(),found_path.end());
    // TODO: Go to parent->parent->parent....
    return found_path;
}

// RRT* methods Everything below is RRT*
double RRT::cost(std::vector<Node> &tree, Node &node)
{
    double cost = 0;
    Node curr_node = node;
    int save = 0;
    while (!curr_node.is_root)
    {
        cost += line_cost(curr_node, tree[curr_node.parent]);
        curr_node = tree[curr_node.parent];
        save = curr_node.parent;
    }
    return cost;
}

double RRT::line_cost(Node &n1, Node &n2)
{
    return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}

double RRT::angle_cost(Node &node, vector<double> &p)
{
    if (!node.is_root)
    {
        pair<float,float> vec1;
        pair<float,float> vec2;
        vec1.first = p[0] - node.x;
        vec1.second = p[1] - node.y;
        Node par_node = tree[node.parent];
        vec2.first = node.x -par_node.x;
        vec2.second = node.y - par_node.y;
        float angle = (atan2(vec2.second, vec2.first)-atan2(vec1.second, vec1.first));
        return angle;
    }
}

double RRT::angle_cost(std::vector<Node> &tree, Node &node, Node &p)
{
    if (!node.is_root)
    {
        pair<float,float> vec1;
        pair<float,float> vec2;
        vec1.first = p.x - node.x;
        vec1.second = p.y - node.y;
        Node par_node = tree[node.parent];
        vec2.first = node.x -par_node.x;
        vec2.second = node.y - par_node.y;
        float angle = acos((vec1.first*vec2.first + vec1.second*vec2.second)/(sqrt(pow(vec1.first,2)+pow(vec1.second,2))+sqrt(pow(vec2.first,2)+pow(vec2.second,2))));
        return angle;
    }
    return 0;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node)
{
    std::vector<int> neighborhood;
    for (int ii = 0; ii < tree.size(); ++ii)
    {
        if (line_cost(tree[ii], node) < rrt_radius_sq)
        {
            neighborhood.push_back(ii);
        }
    }
    return neighborhood;
}

// RRT* end
void RRT::publish_marker(float x, float y)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "current";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    goal_pub.publish(marker);
}

visualization_msgs::Marker RRT::gen_path_marker(const vector<pair<float,float>> &path, float r, float g, float b)
{
    vector<geometry_msgs::Point> tree_points = in_order_path(path);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "current";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.points = tree_points;
    marker.scale.x = 0.05;
    marker.scale.y = 0;
    marker.scale.z = 0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
    //return gen_markers(node_coords, 0, 1, 0);
}

vector<geometry_msgs::Point> RRT::in_order_path(const vector<pair<float,float>> &path)
{
    vector<geometry_msgs::Point> path_points;
    for (int ii = 0; ii < path.size(); ++ii)
    {
        geometry_msgs::Point curr;
        curr.x = path[ii].first;
        curr.y = path[ii].second;
        curr.z = 0.15;
        path_points.push_back(curr);
    }
    return path_points;
}

visualization_msgs::Marker RRT::gen_tree_marker(const vector<Node> &tree, float r, float g, float b)
{
    vector<geometry_msgs::Point> tree_points = full_tree(tree);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "current";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.points = tree_points;
    marker.scale.x = 0.05;
    marker.scale.y = 0;
    marker.scale.z = 0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
    //return gen_markers(node_coords, 0, 1, 0);
}

vector<geometry_msgs::Point> RRT::full_tree(const vector<Node> &tree)
{
    vector<geometry_msgs::Point> tree_points;
    for (int ii = 1; ii < tree.size(); ++ii)
    {
        geometry_msgs::Point one;
        one.x = tree[ii].x;
        one.y = tree[ii].y;
        one.z = 0.1;
        tree_points.push_back(one);
        geometry_msgs::Point two;
        two.x = tree[tree[ii].parent].x;
        two.y = tree[tree[ii].parent].y;
        two.z = 0.1;
        tree_points.push_back(two);
    }
    return tree_points;
}

visualization_msgs::MarkerArray RRT::gen_RRT_markers()
{
    vector<pair<float, float>> node_coords;
    for (int ii = 0; ii < tree.size(); ++ii)
    {
        node_coords.push_back(pair<float,float>(tree[ii].x, tree[ii].y));
    }
    return gen_markers(node_coords, 0, 1, 0);
}

visualization_msgs::MarkerArray RRT::gen_markers(const vector<pair<float,float>> &points, float r, float g, float b)
{
    visualization_msgs::MarkerArray markerArray;
    for (int ii = 0; ii < points.size(); ii += 1)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = ii;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = points[ii].first;
        marker.pose.position.y = points[ii].second;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        markerArray.markers.push_back(marker);
    }
    return markerArray;
}

bool RRT::isPathCollisionFree(vector<pair<float,float>> &path)
{
    for (int i = 1; i < path.size(); i++)
    {
        if (!check_collision(path[i-1],path[i]))
        {
            return false;
        }
    }
    return true;
}

bool RRT::isPathCollisionFree()
{
    if (rrtPath.size()>1)
    {
        for (int i = 1; i < rrtPath.size(); i++)
        {
            Node a(rrtPath[i-1].first, rrtPath[i-1].second, false);
            Node b(rrtPath[i].first, rrtPath[i].second, false);
            if (!check_collision(a, b))
            {
                return false;
            }
        }
    }
    else
    {
        return false;
    }
    return true;
}