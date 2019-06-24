#include "2d_obstacles.hpp"
#include <gtest/gtest.h>



#define DIFF 0.0000001

using namespace Obstacles2D;

TEST(TestObstacles, rect_validity)
{
    Rect r = Rect(0,0,1,1);
    EXPECT_TRUE(r.isValid(std::vector<double>{1.5, 1.5}));
    EXPECT_TRUE(!r.isValid(std::vector<double>{0.5, 0.5}));
}

TEST(TestObstacles, rect_distance)
{
    {
        //x ranges from 1 to 2
        //y ranges from 1 to 2
        Rect r = Rect(1, 1, 2, 2);

        EXPECT_EQ(r.distance(std::vector<double>{1.5, 1.5}), 0.0) << "Point inside rect not 0 distance";
        EXPECT_EQ(r.distance(std::vector<double>{1, 0}), 1.0);
        EXPECT_EQ(r.distance(std::vector<double>{1.5, 0}), 1.0);
        EXPECT_EQ(r.distance(std::vector<double>{1.5, 3}), 1.0);
        EXPECT_EQ(r.distance(std::vector<double>{0, 1.5}), 1.0);
        EXPECT_EQ(r.distance(std::vector<double>{3, 1.5}), 1.0);
        EXPECT_EQ(r.distance(std::vector<double>{0, 1}), 1.0);
        EXPECT_EQ(r.distance(std::vector<double>{0, 0}), std::sqrt(2.0));
        EXPECT_EQ(r.distance(std::vector<double>{3, 0}), std::sqrt(2.0));
        EXPECT_EQ(r.distance(std::vector<double>{0, 3}), std::sqrt(2.0));
        EXPECT_EQ(r.distance(std::vector<double>{3, 3}), std::sqrt(2.0));
    }
    {
        //x ranges from -1 to 0
        // y ranges from 2, 4
        Rect r = Rect(-1, 2, 0, 4);
        EXPECT_EQ(r.distance(std::vector<double>{-0.8, 2.1}), 0.0) << "Point inside rect not 0 distance";
        EXPECT_EQ(r.distance(std::vector<double>{0, 0}), 2.0);
        EXPECT_EQ(r.distance(std::vector<double>{0, 0}), 2.0);
        EXPECT_EQ(r.distance(std::vector<double>{3, 2.1}), 3.0);
        EXPECT_EQ(r.distance(std::vector<double>{-1.5, 0.5}), std::sqrt(0.5*0.5 + 1.5*1.5));
    }
}

TEST(TestObstacles, obstacles)
{
    Obstacles o;
    auto r = std::make_shared<Obstacles2D::Rect>(0.2,0.5,0.3,0.6);
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

