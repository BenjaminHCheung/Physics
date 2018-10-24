#ifndef SPHEREOBJECT_H
#define SPHEREOBJECT_H

#include "PhysicsObject.h"

class SphereObject: public PhysicsObject
{
public:
    SphereObject(Vector3d inputPosition,
                 Vector3d inputVelocity,
                 Vector3d inputAcceleration,
                 double inputCr,
                 double inputMass,
                 double inputRed,
                 double inputGreen,
                 double inputBlue,
                 double inputRadius);

    void set_radius(double newRadius);
    double get_radius();
    double get_area();
    double get_coefficent_of_drag();

private:
    double mRadius{0};
    double mArea{0};
    double mCoefficentOfDrag{.47};

    double calculate_area();
};

#endif // SPHEREOBJECT_H
