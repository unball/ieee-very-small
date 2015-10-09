#include <unball/utils/vector.hpp>
#include <ros/ros.h>

#include <gtest/gtest.h>

TEST(TestAngleClass, testZero)
{
    Vector origin;
    Vector destination;
    
    EXPECT_FLOAT_EQ((origin-destination).getDirection(),0);

    origin.set(1,0);
    EXPECT_FLOAT_EQ((origin-destination).getDirection(),0);
}

TEST(TestAngleClass, testPi_4)
{
    Vector origin(1,1);
    Vector destination;
    
    EXPECT_FLOAT_EQ((origin-destination).getDirection(),M_PI/4);        
}

TEST(TestAngleClass, testPi_2)
{
    Vector origin(0,1);
    Vector destination;
    
    EXPECT_FLOAT_EQ((origin-destination).getDirection(),M_PI/2);
}

TEST(TestAngleClass, test3Pi_4)
{
    Vector origin(-1,1);
    Vector destination;
    
    EXPECT_FLOAT_EQ((origin-destination).getDirection(),3*M_PI/4);        
}

TEST(TestAngleClass, testPi)
{
    Vector origin(-1,0);
    Vector destination;
    
    EXPECT_FLOAT_EQ((origin-destination).getDirection(),M_PI);        
}

TEST(TestAngleClass, testNegativePi_4)
{
    Vector origin;
    Vector destination(-1,1);
    
    EXPECT_FLOAT_EQ((origin-destination).getDirection(),-M_PI/4);    
}

TEST(TestAngleClass, testNegativePi_2)
{
    Vector origin;
    Vector destination(0,1);
    
    EXPECT_FLOAT_EQ((origin-destination).getDirection(),-M_PI/2);
}

TEST(TestAngleClass, testNegative3Pi_4)
{
    Vector origin;
    Vector destination(1,1);
    
    EXPECT_FLOAT_EQ((origin-destination).getDirection(),-3*M_PI/4);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}