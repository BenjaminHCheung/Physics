#include "physicsspace.h"

#include <random>

PhysicsSpace::PhysicsSpace(){}

PhysicsSpace::~PhysicsSpace()
{

}
void PhysicsSpace::set_number_of_objects(unsigned int newNumber)
{
    clear_object_list();
    mNumberOfObjects = newNumber;
    mObjectList.resize(mNumberOfObjects);
    create_physics_objects();
}
void PhysicsSpace::set_max_radius(double maxRadius)
{
    mRadiusMax = maxRadius;
    update_after_change();
}
void PhysicsSpace::set_min_radius(double minRadius)
{
    mRadiusMin = minRadius;
    update_after_change();
}
void PhysicsSpace::set_max_mass(double maxMass)
{
    mMassMax = maxMass;
    update_after_change();
}
void PhysicsSpace::set_min_mass(double minMass)
{
    mMassMin = minMass;
    update_after_change();
}
void PhysicsSpace::set_max_Cr(double maxCr)
{
    mCrMax = maxCr;
    update_after_change();
}
void PhysicsSpace::set_min_Cr(double minCr)
{
    mCrMin = minCr;
    update_after_change();
}
void PhysicsSpace::update_after_change()
{
    clear_object_list();
    create_physics_objects();
}

void PhysicsSpace::timestep_object_list()
{
    for(unsigned int incrementor = 0; incrementor < mNumberOfObjects; incrementor++)
    {
        update_object(mObjectList[incrementor]);
    }
}

void PhysicsSpace::update_object(SphereObject* physicsObject)
{
    object_acceleration_update(physicsObject);
    object_velocity_update(physicsObject);
    object_Position_update(physicsObject);
    check_for_collision(physicsObject);
}

void PhysicsSpace::object_acceleration_update(SphereObject* physicsObject)
{
    Vector3d objectDrag{object_drag(physicsObject)};
    Vector3d newAcceleration = mGravity+objectDrag;
    physicsObject->set_acceleration(newAcceleration);
}

Vector3d PhysicsSpace::object_drag(SphereObject* physicsObject)
{
    Vector3d velocity{physicsObject->get_velocity()};
    Vector3d velocitySquared{velocity*velocity};
    double Cd{physicsObject->get_coefficent_of_drag()};
    double area{physicsObject->get_area()};
    double coefficent{Cd*mFluidDensity*area};
    Vector3d dragForce{velocitySquared*coefficent};
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
    Vector3d newPosition{physicsObject->get_position()+additionPosition};
    physicsObject->set_position(newPosition);
}

void PhysicsSpace::check_for_collision(SphereObject* physicsObject)
{
    const double minimumVelocity{.1};
    if(object_wall_collision(physicsObject->get_position().get_x_value(),physicsObject->get_radius()))
    {
        if(abs(physicsObject->get_position().get_x_value()) > minimumVelocity)
        {
        double xPosition{correct_overshoot(physicsObject,physicsObject->get_position().get_x_value())};
        physicsObject->get_position().set_x_value(xPosition);
        }
        else
        {
        double xPosition{settle_object_at_low_velocities(physicsObject,physicsObject->get_position().get_x_value())};
        physicsObject->get_position().set_x_value(xPosition);
        }
        Vector3d xBounce{Vector3d(-1,1,1)};
        fix_velocity_for_bounce(xBounce, physicsObject);
    }
    if(object_wall_collision(physicsObject->get_position().get_y_value(),physicsObject->get_radius()))
    {
        if(abs(physicsObject->get_position().get_y_value()) > minimumVelocity)
        {
        double yPosition{correct_overshoot(physicsObject,physicsObject->get_position().get_y_value())};
        physicsObject->get_position().set_y_value(yPosition);
        }
        else
        {
        double yPosition{settle_object_at_low_velocities(physicsObject,physicsObject->get_position().get_y_value())};
        physicsObject->get_position().set_y_value(yPosition);
        }
        Vector3d yBounce{Vector3d(1,-1,1)};
        fix_velocity_for_bounce(yBounce, physicsObject);
    }
    if(object_wall_collision(physicsObject->get_position().get_z_value(),physicsObject->get_radius()))
    {
        if(abs(physicsObject->get_position().get_z_value()) > minimumVelocity)
        {
        double zPosition{correct_overshoot(physicsObject,physicsObject->get_position().get_z_value())};
        physicsObject->get_position().set_z_value(zPosition);
        }
        else
        {
        double zPosition{settle_object_at_low_velocities(physicsObject,physicsObject->get_position().get_z_value())};
        physicsObject->get_position().set_z_value(zPosition);
        }
        Vector3d zBounce{Vector3d(1,1,-1)};
        fix_velocity_for_bounce(zBounce, physicsObject);
    }
}

bool PhysicsSpace::object_wall_collision(double position, double radius)
{
    if(position > (mBoxDimension - radius))
    {
        return true;
    }
    else
    {
        return false;
    }
}

double PhysicsSpace::correct_overshoot(SphereObject* physicsObject, double outsidePosition)
{
    double farthestPosition = (mBoxDimension - physicsObject->get_radius());
    double overshoot{abs(outsidePosition) - farthestPosition};
    double trueSign{outsidePosition/abs(outsidePosition)};
    double oppositeSign{(-1)*outsidePosition/abs(outsidePosition)*physicsObject->get_Cr()};
    double newPosition{oppositeSign*overshoot + trueSign*farthestPosition};
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
    double Cr{physicsObject->get_Cr()};
    Vector3d oldVelocity{physicsObject->get_velocity()};
    Vector3d bounceVelocity{bounce*oldVelocity};
    Vector3d newVelocity{bounceVelocity*Cr};
    physicsObject->set_velocity(newVelocity);
}

std::vector<SphereObject*>* PhysicsSpace::get_object_list()
{
    return &mObjectList;
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

        SphereObject* newObject = new SphereObject(position, velocity, mGravity, Cr, mass, color[0], color[1], color[2], radius);
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
    double max{6.0};
    double min{-6.0};
    double xVelocity{generate_random_double(min,max)};
    double yVelocity{generate_random_double(min,max)};
    double zVelocity{generate_random_double(min,max)};
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
    for(unsigned int incrementor = (mNumberOfObjects); incrementor > 0; incrementor--)
    {
        delete mObjectList[incrementor-1];
        mObjectList.pop_back();
    }
}

double PhysicsSpace::generate_random_double(double minValue, double maxValue)
{
    std::uniform_real_distribution<double> unif(minValue,maxValue);
    std::default_random_engine randomGenerator;
    double randNumber{unif(randomGenerator)};
    return randNumber;
}
