#include <unball/utils/math.hpp>
#include <ros/ros.h>

#include <gtest/gtest.h>

TEST(TestInvertAnglesClass, testZero)
{
    EXPECT_FLOAT_EQ(math::invertAngle(0),M_PI);
}

TEST(TestInvertAnglesClass, testPI_4)
{
    EXPECT_FLOAT_EQ(math::invertAngle(M_PI/4),-3*M_PI/4);
}

TEST(TestInvertAnglesClass, testPI_2)
{
    EXPECT_FLOAT_EQ(math::invertAngle(M_PI/2),-M_PI/2);
}

TEST(TestInvertAnglesClass, test3PI_4)
{
    EXPECT_FLOAT_EQ(math::invertAngle(3*M_PI/4),-M_PI/4);
}

TEST(TestInvertAnglesClass, testPI)
{
    EXPECT_FLOAT_EQ(math::invertAngle(M_PI),0);
}

TEST(TestInvertAnglesClass, testNegativePI_4)
{
    EXPECT_FLOAT_EQ(math::invertAngle(-M_PI/4),3*M_PI/4);
}

TEST(TestInvertAnglesClass, testNegativePI_2)
{
    EXPECT_FLOAT_EQ(math::invertAngle(-M_PI/2),M_PI/2);
}

TEST(TestInvertAnglesClass, testNegative3PI_4)
{
    EXPECT_FLOAT_EQ(math::invertAngle(-3*M_PI/4),M_PI/4);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}