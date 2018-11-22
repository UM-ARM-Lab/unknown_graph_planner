// Bring in my package's API, which is what I'm testing
#include "halton.hpp"
// Bring in gtest
#include <gtest/gtest.h>

#define DIFF 0.0000001

// Declare a test
TEST(TestSuite, halton_ind)
{
    EXPECT_EQ(haltonElement(0, 2), 0.0);
    EXPECT_EQ(haltonElement(1, 2), 0.5);
    EXPECT_EQ(haltonElement(2, 2), 0.25);
    EXPECT_EQ(haltonElement(3, 2), 0.75);
    EXPECT_EQ(haltonElement(9, 2), 0.5625);
    EXPECT_NEAR(haltonElement(1, 3), 1.0/3, DIFF);
}



// Declare another test
TEST(TestSuite, hatlon_seq)
{
    auto seq = haltonSeq(2, 5, 0);
    ASSERT_EQ(seq.size(), 5);
    EXPECT_EQ(seq[0], 0);
    EXPECT_EQ(seq[3], 0.75);

    seq = haltonSeq(2, 10000, 3);
    ASSERT_EQ(seq.size(), 10000);
    EXPECT_EQ(seq[0], 0.75); //From matlab
    EXPECT_EQ(seq[1000 - 3], 0.092773437500000); //From matlab
}

TEST(TestSuite, hatlon_points)
{
    std::vector<int> bases{2, 3, 5};
    std::vector<int> offsets{1,2,3};
    auto points = haltonPoints(bases, 100, offsets);
    EXPECT_EQ(points[0][0], 0.5); //base 2, position 1+0
    EXPECT_NEAR(points[0][1], 2.0/3, DIFF); //base 3, position 2+0
    EXPECT_NEAR(points[0][2], 0.6, DIFF); //base 5, position 3+0
    EXPECT_NEAR(points[1][0], 0.25, DIFF); //base 2, position 1+1
    EXPECT_NEAR(points[2][2], 0.04, DIFF); //base 5, position 3+2
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    // ros::init(argc, argv, "tester");
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
