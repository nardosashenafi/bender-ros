#include <pluginlib/class_list_macros.h>
#include <bender_perception/costmap_lane_layer.h>

namespace bender_perception
{

void LaneLayer::onInitialize()
{}

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
