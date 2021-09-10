#ifndef BENDER_PERCEPTION_COSTMAP_LANE_LAYER_H_
#define BENDER_PERCEPTION_COSTMAP_LANE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

namespace bender_perception
{

class LaneLayer : public costmap_2d::CostmapLayer
{
    public:
        LaneLayer()
        {
            costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
        }

        ~LaneLayer();
        void onInitialize() override;
        void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                            double* min_x, double* min_y,
                            double* max_x, double* max_y) override;
        void updateCosts(costmap_2d::Costmap2D& master_grid, 
                            int min_i, int min_j, 
                            int max_i, int max_j) override;
        void activate() override;
        void deactivate() override;
        void reset() override;
    
    protected:
        std::string global_frame_;
        bool rolling_window_;
};

} // namespace bender_perception

#endif // BENDER_PERCEPTION_COSTMAP_LANE_LAYER_H_