#include "gtest/gtest.h"
#include "physicsspace.h"

TEST(GivenTwoObjectsWithSetPositions, WhenUsingMagnitudeFunction_ReturnsCorrectMagnitude)
{
    PhysicsSpace* physicsSpace = new PhysicsSpace();
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

TEST(GivenCollisionOfTwoObjectsOfEqualMassVelocityAndDirection, WhenUsingTheCollisionFunction_ReturnsTheCorrectVelocitiesForBothObjects)
{
    PhysicsSpace* physicsSpace = new PhysicsSpace();
    Vector3d firstObjectPosition{Vector3d(0.0,0.0,0.0)};
    Vector3d secondObjectPosition{Vector3d(2.0,0.0,0.0)};

    Vector3d firstObjectVelocity{Vector3d(10.0,0.0,0.0)};
    Vector3d secondObjectVelocity{Vector3d(-10.0,0.0,0.0)};

    double firstObjectMass{5.0};
    double secondObjectMass{5.0};

    Vector3d zeroVector{Vector3d(0.0,0.0,0.0)};

    SphereObject* firstObject = new SphereObject(firstObjectPosition,
                                                 zeroVector,
                                                 zeroVector,
                                                 1.0, firstObjectMass, 1.0, 1.0, 1.0, 1.0);
    SphereObject* secondObject = new SphereObject(secondObjectPosition,
                                                  zeroVector,
                                                  zeroVector,
                                                  1.0, secondObjectMass, 1.0, 1.0, 1.0, 1.0);
    physicsSpace->two_object_collision(firstObject, secondObject, physicsSpace->calculate_magnitude_of_position_vectors(firstObject,secondObject));
    double firstObjectXVelocity{firstObject->get_velocity().get_x_value()};
    double expectedFirstObjectXVelocity{secondObjectVelocity.get_x_value()};
    EXPECT_NEAR(firstObjectXVelocity, expectedFirstObjectXVelocity, .00001);

}
