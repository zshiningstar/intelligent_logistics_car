
#include "ground_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");

    ros::NodeHandle nh;

    PclTestCore core(nh);
    // core.Spin();
    return 0;
}
