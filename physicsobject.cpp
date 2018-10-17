#include "physicsobject.h"

PhysicsObject::PhysicsObject(Vector3d inputPosition,
                             Vector3d inputVelocity,
                             Vector3d inputAcceleration,
                             double inputCr,
                             double inputMass,
                             double inputRed,
                             double inputGreen,
                             double inputBlue)
{
    mPosition = inputPosition;
    mVelocity = inputVelocity;
    mAcceleration = inputAcceleration;
    mCr = inputCr;
    mMass = inputMass;
    mColor[0] = inputRed;
    mColor[1] = inputGreen;
    mColor[2] = inputBlue;
}
void PhysicsObject::set_position(Vector3d newPosition)
{
    mPosition = newPosition;
}

void PhysicsObject::set_velocity(Vector3d newVelocity)
{
    mVelocity = newVelocity;
}

void PhysicsObject::set_acceleration(Vector3d newAcceleration)
{
    mAcceleration = newAcceleration;
}

void PhysicsObject::set_Cr(double newCoefficnetOfRestitution)
{
    mCr = newCoefficnetOfRestitution;
}

void PhysicsObject::set_mass(double newMass)
{
    mMass = newMass;
}

void PhysicsObject::set_color(double red, double green, double blue)
{
    mColor[0] = red;
    mColor[1] = green;
    mColor[2] = blue;
}

Vector3d PhysicsObject::get_position()
{
    return mPosition;
}

Vector3d PhysicsObject::get_velocity()
{
    return mVelocity;
}

Vector3d PhysicsObject::get_acceleration()
{
    return mAcceleration;
}

double PhysicsObject::get_Cr()
{
    return mCr;
}

double PhysicsObject::get_mass()
{
    return mMass;
}

double* PhysicsObject::get_color()
{
    return mColor;
}
