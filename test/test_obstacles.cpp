#include "2d_obstacles.hpp"
#include <gtest/gtest.h>



#define DIFF 0.0000001

using namespace Obstacles2D;

TEST(TestObstacles, rect)
{
    Rect r = Rect(0,0,1,1);
    EXPECT_TRUE(r.isValid(std::vector<double>{1.5, 1.5}));
    EXPECT_TRUE(!r.isValid(std::vector<double>{0.5, 0.5}));
}

TEST(TestObstacles, obstacles)
{
    Obstacles o;
    Obstacles2D::Rect* r = new Obstacles2D::Rect(0.2,0.5,0.3,0.6);
    o.obs.push_back(r);

    EXPECT_TRUE(o.isValid(std::vector<double>{1.5, 1.5}));
    EXPECT_FALSE(o.isValid(std::vector<double>{0.25, 0.55}));
    EXPECT_TRUE(o.isValid(std::vector<double>{0.25, 0.65}));
    EXPECT_TRUE(o.isValid(std::vector<double>{0.35, 0.55}));
    EXPECT_TRUE(o.isValid(std::vector<double>{0.15, 0.55}));
    EXPECT_TRUE(o.isValid(std::vector<double>{0.25, 0.45}));
    EXPECT_TRUE(o.isValid(std::vector<double>{0.55, 0.25}));
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

