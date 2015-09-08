#include <unball/utils/test_angles.hpp>

bool TestAngles::run()
{
    bool passed = true;
    if (testZero() == false)
        passed = false;
    if (testPI_4() == false)
        passed = false;
    if (testPI_2() == false)
        passed = false;
    if (test3PI_4() == false)
        passed = false;
    if (testPI() == false)
        passed = false;    
    if (testNegativePI_4() == false)
        passed = false;
    if (testNegativePI_2() == false)
        passed = false;
    if (testNegative3PI_4() == false)
        passed = false;
    if (testNegativePI() == false)
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
    Vector origin;
    Vector destination;
    bool passed;

    //PI/4
    passed = verifyAngle(origin,destination,M_PI/4);
    if (not passed) {
        ROS_ERROR("[TestAngless]testPI_4: failed");
        return passed;
    }   

    return true;
}

bool TestAngles::testPI_2()
{
    Vector origin;
    Vector destination;
    bool passed;

    //PI/2
    passed = verifyAngle(origin,destination,M_PI/2);
    if (not passed) {
        ROS_ERROR("[TestAngless]testPI_2: failed");
        return passed;
    }   

    return true;
}

bool TestAngles::test3PI_4()
{
    Vector origin;
    Vector destination;
    bool passed;

    //3PI/4
    passed = verifyAngle(origin,destination,3*M_PI/4);
    if (not passed) {
        ROS_ERROR("[TestAngless]test3PI_4: failed");
        return passed;
    }   

    return true;
}

bool TestAngles::testPI()
{
    Vector origin;
    Vector destination;
    bool passed;

    //PI
    passed = verifyAngle(origin,destination,M_PI);
    if (not passed) {
        ROS_ERROR("[TestAngless]testPI: failed");
        return passed;
    }

    return true;
}

bool TestAngles::testNegativePI_4()
{
    Vector origin;
    Vector destination;
    bool passed;

    //PI/4
    passed = verifyAngle(origin,destination,-M_PI/4);
    if (not passed) {
        ROS_ERROR("[TestAngless]testNegativePI_4: failed");
        return passed;
    }   

    return true;
}

bool TestAngles::testNegativePI_2()
{
    Vector origin;
    Vector destination;
    bool passed;

    //PI/4
    passed = verifyAngle(origin,destination,-M_PI/2);
    if (not passed) {
        ROS_ERROR("[TestAngless]testNegativePI_2: failed");
        return passed;
    }   

    return true;
}

bool TestAngles::testNegative3PI_4()
{
    Vector origin;
    Vector destination;
    bool passed;

    //PI/4
    passed = verifyAngle(origin,destination,-3*M_PI/4);
    if (not passed) {
        ROS_ERROR("[TestAngless]testNegative3PI_4: failed");
        return passed;
    }   

    return true;
}

bool TestAngles::testNegativePI()
{
    Vector origin;
    Vector destination;
    bool passed;

    //PI/4
    passed = verifyAngle(origin,destination,-M_PI);
    if (not passed) {
        ROS_ERROR("[TestAngless]testNegativePI: failed");
        return passed;
    }   

    return true;
}

bool TestAngles::verifyAngle(Vector origin, Vector destination, float tested_angle)
{
    Vector difference = destination - origin;
    float angle = difference.getDirection();
    float error = fabs(angle - tested_angle);
    return(error < 0.5);
}
