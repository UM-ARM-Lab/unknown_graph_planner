// Bring in my package's API, which is what I'm testing
#include "lpa_pqueue.hpp"
#include "dijkstras_addons.hpp"
// Bring in gtest
#include <gtest/gtest.h>

#define DIFF 0.0000001


TEST(DIJKSTRAS_ADDONS, queue)
{
    arc_dijkstras::pqueue<double, int64_t> queue;

    queue.insert({1.123, 2});

    EXPECT_FALSE(queue.isEmpty());
    EXPECT_EQ(queue.top().first, 1.123);
    EXPECT_EQ(queue.top().first, 1.123);
    EXPECT_EQ(queue.top().second, 2);
    EXPECT_EQ(queue.top().second, 2);

    queue.insert({1.101, 1});
    EXPECT_EQ(queue.top().first, 1.101);
    EXPECT_EQ(queue.top().second, 1);

    size_t num_erased = queue.remove(1);
    EXPECT_EQ(num_erased, 1);
    EXPECT_EQ(queue.top().first, 1.123);
    EXPECT_EQ(queue.top().second, 2);

    num_erased = queue.remove(1);
    EXPECT_EQ(num_erased, 0);
    EXPECT_EQ(queue.top().first, 1.123);
    EXPECT_EQ(queue.top().second, 2);



    queue.insert({1.345, 3});
    EXPECT_EQ(queue.top().second, 2);

    queue.insert({1.00, 3});
    EXPECT_EQ(queue.top().second, 3);

    num_erased = queue.remove(3);
    EXPECT_EQ(num_erased, 2);
    EXPECT_EQ(queue.top().second, 2);

    num_erased = queue.remove(2);
    EXPECT_EQ(num_erased, 1);
    EXPECT_TRUE(queue.isEmpty());
}

TEST(DIJKSTRAS_ADDONS, LPA)
{
    GraphD g;
    g.addNode(std::vector<double>{0,0});
    g.addNode(std::vector<double>{0.1,0.1});
}




// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
