#include "gtest/gtest.h"
#include "physicsspace.h"

TEST(GivenTwoObjectsWithSetPositions, WhenUsingMagnitudeFunction_ReturnsCorrectMagnitude)
{
    PhysicsSpace* physicsSpace = new PhysicsSpace();
    physicsSpace->create_physics_objects();
    Vector3d firstObjectPosition{Vector3d(-3.0,2.5,1.1)};
    Vector3d secondObjectPosition{Vector3d(2.5,.0,1.1)};

    Vector3d zeroVector{Vector3d(0.0,0.0,0.0)};

    SphereObject* firstObject = new SphereObject(firstObjectPosition,
                                                 zeroVector,
                                                 zeroVector,
                                                 1.0, 2.0, 1.0, 1.0, 1.0, 1.0);
    SphereObject* secondObject = new SphereObject(secondObjectPosition,
                                                  zeroVector,
                                                  zeroVector,
                                                  1.0, 2.0, 1.0, 1.0, 1.0, 1.0);
    double expectedMagnitude{6.041523};
    double calculatedMagnitude{physicsSpace->calculate_magnitude_of_position_vectors(firstObject,secondObject)};
    EXPECT_NEAR(expectedMagnitude, calculatedMagnitude, .00001);
    delete physicsSpace;
    delete firstObject;
    delete secondObject;
}

TEST(GivenCollisionOfTwoObjectsOfEqualMassVelocityAndOppositeDirection, WhenUsingTheCollisionFunction_ReturnsTheCorrectVelocitiesForBothObjects)
{
    PhysicsSpace* physicsSpace = new PhysicsSpace();
    physicsSpace->create_physics_objects();

    Vector3d firstObjectPosition{Vector3d(-1.0,0.0,0.0)};
    Vector3d secondObjectPosition{Vector3d(1.0,0.0,0.0)};

    Vector3d firstObjectVelocity{Vector3d(10.0,0.0,0.0)};
    Vector3d secondObjectVelocity{Vector3d(-10.0,0.0,0.0)};

    double firstObjectMass{5.0};
    double secondObjectMass{5.0};

    Vector3d zeroVector{Vector3d(0.0,0.0,0.0)};

    SphereObject* firstObject = new SphereObject(firstObjectPosition,
                                                 firstObjectVelocity,
                                                 zeroVector,
                                                 1.0, firstObjectMass, 1.0, 1.0, 1.0, 1.0);
    SphereObject* secondObject = new SphereObject(secondObjectPosition,
                                                  secondObjectVelocity,
                                                  zeroVector,
                                                  1.0, secondObjectMass, 1.0, 1.0, 1.0, 1.0);
    physicsSpace->do_objects_collide(firstObject, secondObject);
    Vector3d newFirstObjectVelocity{firstObject->get_velocity()};
    Vector3d expectedFirstObjectVelocity{Vector3d(-10.0,0,0)};
    EXPECT_DOUBLE_EQ(newFirstObjectVelocity.get_x_value(), expectedFirstObjectVelocity.get_x_value());
    EXPECT_DOUBLE_EQ(newFirstObjectVelocity.get_y_value(), expectedFirstObjectVelocity.get_y_value());
    EXPECT_DOUBLE_EQ(newFirstObjectVelocity.get_z_value(), expectedFirstObjectVelocity.get_z_value());
    Vector3d newSecondObjectVelocity{secondObject->get_velocity()};
    Vector3d expectedSecondObjectXVelocity{Vector3d(10.0,0,0)};
    EXPECT_DOUBLE_EQ(newSecondObjectVelocity.get_x_value(), expectedSecondObjectXVelocity.get_x_value());
    EXPECT_DOUBLE_EQ(newSecondObjectVelocity.get_y_value(), expectedSecondObjectXVelocity.get_y_value());
    EXPECT_DOUBLE_EQ(newSecondObjectVelocity.get_z_value(), expectedSecondObjectXVelocity.get_z_value());
    delete physicsSpace;
    delete firstObject;
    delete secondObject;
}

TEST(GivenCollisionBetweenTwoOverLapingObjects, WhenUsingTheCollisionEquation_CorrectlyRepositionsTheSpheres)
{
    PhysicsSpace* physicsSpace = new PhysicsSpace();
    physicsSpace->create_physics_objects();

    Vector3d firstObjectPosition{Vector3d(-0.1,-0.1,-0.1)};
    Vector3d secondObjectPosition{Vector3d(0.1,0.1,0.1)};

    Vector3d firstObjectVelocity{Vector3d(10.0,0.0,0.0)};
    Vector3d secondObjectVelocity{Vector3d(-10.0,0.0,0.0)};

    double firstObjectMass{5.0};
    double secondObjectMass{5.0};

    Vector3d zeroVector{Vector3d(0.0,0.0,0.0)};

    SphereObject* firstObject = new SphereObject(firstObjectPosition,
                                                 firstObjectVelocity,
                                                 zeroVector,
                                                 1.0, firstObjectMass, 1.0, 1.0, 1.0, 1.0);
    SphereObject* secondObject = new SphereObject(secondObjectPosition,
                                                  secondObjectVelocity,
                                                  zeroVector,
                                                  1.0, secondObjectMass, 1.0, 1.0, 1.0, 1.0);

    physicsSpace->do_objects_collide(firstObject, secondObject);

    double expectedValue{.6728203230};
    Vector3d newFirstObjectPosition{firstObject->get_position()};
    Vector3d expectedFirstObjectPosition{Vector3d(-expectedValue,-expectedValue,-expectedValue)};
    EXPECT_NEAR(newFirstObjectPosition.get_x_value(), expectedFirstObjectPosition.get_x_value(), .0000001);
    EXPECT_NEAR(newFirstObjectPosition.get_y_value(), expectedFirstObjectPosition.get_y_value(), .0000001);
    EXPECT_NEAR(newFirstObjectPosition.get_z_value(), expectedFirstObjectPosition.get_z_value(), .0000001);
    Vector3d newSecondObjectPosition{secondObject->get_position()};
    Vector3d expectedSecondObjectXPosition{Vector3d(expectedValue,expectedValue,expectedValue)};
    EXPECT_NEAR(newSecondObjectPosition.get_x_value(), expectedSecondObjectXPosition.get_x_value(), .0000001);
    EXPECT_NEAR(newSecondObjectPosition.get_y_value(), expectedSecondObjectXPosition.get_y_value(), .0000001);
    EXPECT_NEAR(newSecondObjectPosition.get_z_value(), expectedSecondObjectXPosition.get_z_value(), .0000001);
    delete physicsSpace;
    delete firstObject;
    delete secondObject;

}
//test functions that are public facing need to have unit tests
