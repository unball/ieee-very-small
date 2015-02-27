/**
 * @file   test_point.cpp
 * @author Matheus Vieira Portela
 * @date   24/07/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Tests for point class
 */

#include <gtest/gtest.h>

#include <unball/utils/point.hpp>

TEST(TestPointClass, testDefaultConstructor)
{
    Point p;
    EXPECT_EQ(p.getX(), 0);
    EXPECT_EQ(p.getY(), 0);
}

TEST(TestPointClass, testConstructorWithParameters)
{
    Point p(10, 20);
    EXPECT_EQ(p.getX(), 10);
    EXPECT_EQ(p.getY(), 20);
}

TEST(TestPointClass, testSet1)
{
    Point p;
    p.set(10, 20);
    EXPECT_EQ(p.getX(), 10);
    EXPECT_EQ(p.getY(), 20);
}

TEST(TestPointClass, testSet2)
{
    Point p;
    p.set(-10, -20);
    EXPECT_EQ(p.getX(), -10);
    EXPECT_EQ(p.getY(), -20);
}

TEST(TestPointClass, testSet3)
{
    Point p;
    p.set(Point(-10, -20));
    EXPECT_EQ(p.getX(), -10);
    EXPECT_EQ(p.getY(), -20);
}

TEST(TestPointClass, testMovePointWithAngle)
{
    Point p;
    p.movePointWithAngle(10, 0);
    EXPECT_EQ(p.getX(), 10);
    EXPECT_EQ(p.getY(), 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}