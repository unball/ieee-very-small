#include <unball/utils/test_angles.hpp>

bool TestAngles::run()
{
    bool passed = true;
    if (testZero() == false)
        passed = false;

    return passed;
};

bool TestAngles::testZero()
{
    Vector origin;
    Vector destination;
    bool passed;

    //zero
    passed = verifyAngle(origin,destination,0);
    if (not passed) {
        ROS_ERROR("[TestAngless]testZero: failed");
        return passed;
    }

    //zero
    destination.set(1,0);
    passed = verifyAngle(origin,destination,0);
    if (not passed) {
        ROS_ERROR("[TestAngless]testZero: failed");
        return passed;
    }

    //pi/2
    destination.set(0,1);
    passed = verifyAngle(origin,destination,0);
    if (passed) {
        ROS_ERROR("[TestAngless]testZero: failed");
        return passed;        
    }

    //pi/4
    destination.set(1,1);
    passed = verifyAngle(origin,destination,0);
    if (passed) {
        ROS_ERROR("[TestAngless]testZero: failed");
        return passed;        
    }

    //pi ou -pi
    destination.set(0,-1);
    passed = verifyAngle(origin,destination,0);
    if (passed) {
        ROS_ERROR("[TestAngless]testZero: failed");
        return passed;        
    }

    return true;
}

bool TestAngles::testPI_4()
{

}

bool TestAngles::testPI_2()
{

}

bool TestAngles::test3PI_4()
{

}

bool TestAngles::testPI()
{

}

bool TestAngles::testNegativePI_4()
{

}

bool TestAngles::testNegativePI_2()
{

}

bool TestAngles::testNegative3PI_4()
{

}

bool TestAngles::testNegativePI()
{

}

bool TestAngles::verifyAngle(Vector origin, Vector destination, float tested_angle)
{
    Vector difference = destination - origin;
    float angle = difference.getDirection();
    float error = fabs(angle - tested_angle);
    return(error < 0.5);
}
