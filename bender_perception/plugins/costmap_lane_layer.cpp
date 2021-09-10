#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <bender_perception/costmap_lane_layer.h>

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace bender_perception
{

void LaneLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_), g_nh;
    rolling_window_ = layered_costmap_->isRolling();

    bool track_unknown_space;
    nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
    if (track_unknown_space)
        default_value_ = NO_INFORMATION;
    else
        default_value_ = FREE_SPACE;

    LaneLayer::matchSize();
    current_ = true;

    // Layer parameters
    global_frame_ = layered_costmap_->getGlobalFrameID();
    double transform_tolerance;
    nh.param("transform_tolerance", transform_tolerance, 0.2);

    std::string topics_string;
    nh.param("observation_sources", topics_string, std::string(""));
    ROS_INFO("    Observation sources are: %s", topics_string.c_str());
    std::stringstream ss(topics_string);

    std::string source;
    while (ss >> source)
    {
        ros::NodeHandle source_node(nh, source);
        // get the parameters for the specific topic
        double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
        std::string topic, sensor_frame, data_type;
        bool inf_is_valid, clearing, marking;
        double FOV_V, FOV_W, min_detect_distance, max_detect_distance;

        source_node.param("topic", topic, source);
        source_node.param("sensor_frame", sensor_frame, std::string(""));
        source_node.param("observation_persistence", observation_keep_time, 0.0);
        source_node.param("expected_update_rate", expected_update_rate, 0.0);
        source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
        source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
        source_node.param("inf_is_valid", inf_is_valid, false);
        source_node.param("clearing", clearing, false);
        source_node.param("marking", marking, true);

        source_node.param("FOV_V", FOV_V, 1.0);
        source_node.param("FOV_W", FOV_W, 1.5);
        source_node.param("min_detect_distance", min_detect_distance, 0.15);
        source_node.param("max_detect_distanxe", max_detect_distance, 2.5);

    }
}

void LaneLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, 
                                double* min_x, double* min_y,
                                double* max_x, double* max_y)
{}

void LaneLayer::updateCosts(costmap_2d::Costmap2D& master_grid, 
                            int min_i, int min_j, 
                            int max_i, int max_j)
{}

void LaneLayer::activate()
{}

void LaneLayer::deactivate()
{}

void LaneLayer::reset()
{}


} // namespace bender_perception

PLUGINLIB_EXPORT_CLASS(bender_perception::LaneLayer, costmap_2d::Layer)
