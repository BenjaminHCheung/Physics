#ifndef PHYSICSOBJECT_H
#define PHYSICSOBJECT_H

#include "vector3d.h"

class PhysicsObject
{
public:
    PhysicsObject(Vector3d inputPosition,
                  Vector3d inputVelocity,
                  Vector3d inputAcceleration,
                  double inputCr,
                  double inputMass,
                  double inputRed,
                  double inputGreen,
                  double inputBlue);

    void set_position(Vector3d newPosition);
    void set_velocity(Vector3d newVelocity);
    void set_acceleration(Vector3d newAcceleration);
    void set_Cr(double newCoefficnetOfRestitution);
    void set_mass(double newMass);
    void set_color(double red, double green, double blue);
    Vector3d get_position();
    Vector3d get_velocity();
    Vector3d get_acceleration();
    double get_Cr();
    double get_mass();
    double* get_color();

private:
    Vector3d mPosition{Vector3d(0,0,0)};
    Vector3d mOldPosition{Vector3d(0,0,0)};
    Vector3d mVelocity{Vector3d(0,0,0)};
    Vector3d mAcceleration{Vector3d(0,0,-9.8)};
    double mMass;
    double mCr;
    double mColor[3];

};

#endif // PHYSICSOBJECT_H
