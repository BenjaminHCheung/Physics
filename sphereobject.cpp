#include "sphereobject.h"

SphereObject::SphereObject(Vector3d inputPosition,
                           Vector3d inputVelocity,
                           Vector3d inputAcceleration,
                           double inputCr,
                           double inputMass,
                           double inputRed,
                           double inputGreen,
                           double inputBlue,
                           double inputRadius)
    :PhysicsObject (inputPosition,
                    inputVelocity,
                    inputAcceleration,
                    inputCr,
                    inputMass,
                    inputRed,
                    inputGreen,
                    inputBlue)
{
    mRadius = inputRadius;
    mArea = calculate_area();
}

void SphereObject::set_radius(double newRadius)
{
    mRadius = newRadius;
}

double SphereObject::get_radius()
{
    return mRadius;
}

double SphereObject::get_area()
{
    return mArea;
}

double SphereObject::get_coefficent_of_drag()
{
    return mCoefficentOfDrag;
}

double SphereObject::calculate_area()
{
    const double pi = 3.14159265358979;
    double Area{ pi * ( mRadius * mRadius )};
    return Area;
}
