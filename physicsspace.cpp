#include "physicsspace.h"

#include <random>
#include <cmath>

PhysicsSpace::PhysicsSpace(){}

PhysicsSpace::~PhysicsSpace()
{
    clear_object_list();
}
void PhysicsSpace::set_number_of_objects(unsigned int newNumber)
{
    mOldNumberOfObjects = mNumberOfObjects;
    mNumberOfObjects = newNumber;
}
void PhysicsSpace::set_gravity(double xValue, double yValue, double zValue)
{
    Vector3d newGravity{Vector3d(xValue, yValue, zValue)};
    mGravity = newGravity;
}
void PhysicsSpace::set_fluid_density(double density)
{
    mFluidDensity = density;
}
void PhysicsSpace::set_max_radius(double maxRadius)
{
    mRadiusMax = maxRadius;
}
void PhysicsSpace::set_min_radius(double minRadius)
{
    mRadiusMin = minRadius;
}
void PhysicsSpace::set_max_mass(double maxMass)
{
    mMassMax = maxMass;
}
void PhysicsSpace::set_min_mass(double minMass)
{
    mMassMin = minMass;
}
void PhysicsSpace::set_max_Cr(double maxCr)
{
    mCrMax = maxCr;
}
void PhysicsSpace::set_min_Cr(double minCr)
{
    mCrMin = minCr;
}
void PhysicsSpace::set_velocity_max(double maxVelocity)
{
    mVelocityMax = maxVelocity;
}
void PhysicsSpace::set_velocity_min(double minVelocity)
{
    mVelocityMin = minVelocity;
}
void PhysicsSpace::update_after_change()
{
    clear_object_list();
    create_physics_objects();
}

void PhysicsSpace::timestep_object_list()
{;
    for(unsigned int incrementor{0} ; incrementor < mNumberOfObjects; incrementor++)
    {
        for(unsigned int Iteration{0}; Iteration < mNumberOfIterations; Iteration++)
        {
            update_object(mObjectList[incrementor]);
        }
    }
}

void PhysicsSpace::update_object(SphereObject* physicsObject)
{
    object_acceleration_update(physicsObject);
    object_velocity_update(physicsObject);
    check_for_collision(physicsObject);
    object_Position_update(physicsObject);
}

void PhysicsSpace::object_acceleration_update(SphereObject* physicsObject)
{
    Vector3d objectDrag{object_drag(physicsObject)};
    Vector3d newAcceleration = mGravity + objectDrag;
    physicsObject->set_acceleration(newAcceleration);
}

Vector3d PhysicsSpace::object_drag(SphereObject* physicsObject)
{
    Vector3d velocity{physicsObject->get_velocity()};
    Vector3d velocitySquared{velocity*velocity};
    Vector3d signOfVelocity{Vector3d(velocity.get_x_value()/abs(velocity.get_x_value()),
                                     velocity.get_y_value()/abs(velocity.get_y_value()),
                                     velocity.get_z_value()/abs(velocity.get_z_value()))};
    Vector3d reverseDirection{signOfVelocity*(-1)};
    double Cd{physicsObject->get_coefficent_of_drag()};
    double area{physicsObject->get_area()};
    double coefficent{Cd*mFluidDensity*area};
    Vector3d dragDirection{velocitySquared*reverseDirection};
    Vector3d dragForce{dragDirection*coefficent};
    double mass{physicsObject->get_mass()};
    Vector3d drag{dragForce/mass};
    return drag;
}
void PhysicsSpace::object_velocity_update(SphereObject* physicsObject)
{
    Vector3d acceleration{physicsObject->get_acceleration()};
    Vector3d additionVelocity{acceleration*mTimeStep};
    Vector3d newVelocity{physicsObject->get_velocity()+additionVelocity};
    physicsObject->set_velocity(newVelocity);
}

void PhysicsSpace::object_Position_update(SphereObject* physicsObject)
{
    Vector3d velocity{physicsObject->get_velocity()};
    Vector3d additionPosition{velocity*mTimeStep};
    Vector3d currentPosition{physicsObject->get_position()};
    Vector3d newPosition{currentPosition+additionPosition};
    physicsObject->set_position(newPosition);
}

void PhysicsSpace::check_for_collision(SphereObject* physicsObject)
{
    if(object_wall_collision(physicsObject->get_position().get_x_value(),physicsObject->get_radius()))
    {
        double xPosition{correct_overshoot(physicsObject->get_radius(),physicsObject->get_position().get_x_value())};
        physicsObject->get_position().set_x_value(xPosition);
        Vector3d xBounce{Vector3d(-physicsObject->get_Cr(),1.0,1.0)};
        fix_velocity_for_bounce(xBounce, physicsObject);
    }
    if(object_wall_collision(physicsObject->get_position().get_y_value(),physicsObject->get_radius()))
    {
        double yPosition{correct_overshoot(physicsObject->get_radius(),physicsObject->get_position().get_y_value())};
        physicsObject->get_position().set_y_value(yPosition);
        Vector3d yBounce{Vector3d(1.0,-physicsObject->get_Cr(),1.0)};
        fix_velocity_for_bounce(yBounce, physicsObject);
    }
    if(object_wall_collision(physicsObject->get_position().get_z_value(),physicsObject->get_radius()))
    {
        double zPosition{correct_overshoot(physicsObject->get_radius(),physicsObject->get_position().get_z_value())};
        physicsObject->get_position().set_z_value(zPosition);
        Vector3d zBounce{Vector3d(1.0,1.0,-physicsObject->get_Cr())};
        fix_velocity_for_bounce(zBounce, physicsObject);
    }
}

bool PhysicsSpace::object_wall_collision(double position, double radius)
{
    return (abs(position)+radius) > mBoxDimension;
}

double PhysicsSpace::correct_overshoot(double radius, double outsidePosition)
{
    double farthestPosition{mBoxDimension - radius - mRadiusSafetyMargin};
    double oversthoot{abs(outsidePosition) - farthestPosition};
    double sign{std::copysign(1.0, outsidePosition)};
    double newPosition{sign*(farthestPosition - oversthoot)};
    return newPosition;
}

double PhysicsSpace::settle_object_at_low_velocities(SphereObject* physicsObject, double steadyPosition)
{
    double signPosition{steadyPosition/abs(steadyPosition)};
    double settlePosition = signPosition*(mBoxDimension-physicsObject->get_radius());
    return settlePosition;
}

void PhysicsSpace::fix_velocity_for_bounce(Vector3d bounce, SphereObject* physicsObject)
{
    Vector3d oldVelocity{physicsObject->get_velocity()};
    Vector3d bounceVelocity{bounce*oldVelocity};
    physicsObject->set_velocity(bounceVelocity);
}

std::vector<SphereObject*> PhysicsSpace::get_object_list()
{
    return mObjectList;
}

unsigned int PhysicsSpace::get_objectlist_size()
{
    return mNumberOfObjects;
}

void PhysicsSpace::create_physics_objects()
{
    for(unsigned int incrementor = 0; incrementor < mNumberOfObjects; incrementor++)
    {
        Vector3d position{generate_random_position()};
        Vector3d velocity{generate_random_velocity()};
        double Cr{generate_random_double(mCrMin,mCrMax)};
        double mass{generate_random_double(mMassMin,mMassMax)};
        double* color{generate_random_color()};
        double radius{generate_random_double(mRadiusMin,mRadiusMax)};
        SphereObject* newObject{new SphereObject(position, velocity, mGravity, Cr, mass, color[0], color[1], color[2], radius)};
        mObjectList.push_back(newObject);
    }
}

Vector3d PhysicsSpace::generate_random_position()
{
    double max{4.0};
    double min{-4.0};
    double xPosition{generate_random_double(min,max)};
    double yPosition{generate_random_double(min,max)};
    double zPosition{generate_random_double(min,max)};
    Vector3d position{Vector3d(xPosition,yPosition,zPosition)};
    return position;
}

Vector3d PhysicsSpace::generate_random_velocity()
{
    double minAngle{-180.0};
    double maxAngle{180.0};
    const double pi = 3.14159265358979;
    double magnitudeVelocity{generate_random_double(mVelocityMin,mVelocityMax)};
    double theta{pi/180.0*generate_random_double(minAngle,maxAngle)};
    double psi{pi/180.0*generate_random_double(minAngle,maxAngle)};
    double xVelocity{magnitudeVelocity*cos(theta)*cos(psi)};
    double yVelocity{magnitudeVelocity*sin(theta)*cos(psi)};
    double zVelocity{magnitudeVelocity*sin(psi)};
    Vector3d velocity{Vector3d(xVelocity,yVelocity,zVelocity)};
    return velocity;
}

double* PhysicsSpace::generate_random_color()
{
    int minColor = 0;
    int maxColor = 1;
    double* color = new double[3];
    double red{generate_random_double(minColor, maxColor)};
    double green{generate_random_double(minColor, maxColor)};
    double blue{generate_random_double(minColor, maxColor)};
    color[0] = red;
    color[1] = green;
    color[2] = blue;
    return color;
}

void PhysicsSpace::clear_object_list()
{
    for(unsigned int incrementor{mOldNumberOfObjects}; incrementor > 0; incrementor--)
    {
        delete mObjectList[(incrementor - 1)];
        mObjectList.pop_back();
    }
}

double PhysicsSpace::generate_random_double(double minValue, double maxValue)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(minValue,maxValue);
    return dis(gen);
}
