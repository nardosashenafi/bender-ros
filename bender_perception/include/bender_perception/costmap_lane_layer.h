#ifndef BENDER_PERCEPTION_COSTMAP_LANE_LAYER_H_
#define BENDER_PERCEPTION_COSTMAP_LANE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

namespace bender_perception
{

class LaneLayer : public costmap_2d::CostmapLayer
{
    LaneLayer()
    {
        costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
    }

    virtual ~LaneLayer();
};

} // namespace bender_perception

#endif // BENDER_PERCEPTION_COSTMAP_LANE_LAYER_H_