#ifndef PHYSICSSPACE_H
#define PHYSICSSPACE_H

#include "vector3d.h"
#include "sphereobject.h"
#include <vector>

class PhysicsSpace
{
public:
    PhysicsSpace();
    ~PhysicsSpace();

    void set_number_of_objects(unsigned int newNumber);
    void set_max_radius(double maxRadius);
    void set_min_radius(double minRadius);
    void set_max_mass(double maxMass);
    void set_min_mass(double minMass);
    void set_max_Cr(double maxCr);
    void set_min_Cr(double minCr);
    void update_after_change();

    void timestep_object_list();
    void update_object(SphereObject* physicsObject);
    void object_acceleration_update(SphereObject* physicsObject);
    Vector3d object_drag(SphereObject* physicsObject);
    void object_velocity_update(SphereObject* physicsObject);
    void object_Position_update(SphereObject* physicsObject);
    void check_for_collision(SphereObject* physicsObject);
    bool object_wall_collision(double position, double radius);
    double correct_overshoot(SphereObject* physicsObject, double largePosition);
    double settle_object_at_low_velocities(SphereObject* physicsObject, double steadyPosition);

private:
    unsigned int mNumberOfObjects{10};
    double mTimeStep{1/30};
    double mRadiusMin{0.2};
    double mRadiusMax{1.0};
    double mMassMin{2.0};
    double mMassMax{10.0};
    double mCrMin{0.2};
    double mCrMax{1.0};
    double mBoxDimension{5.0};
    double mFluidDensity{1.275};
    Vector3d mGravity{Vector3d(0,0,-9.8)};
    std::vector<SphereObject*> mObjectList;

    void create_physics_objects();
    Vector3d generate_random_position();
    Vector3d generate_random_velocity();
    double* generate_random_color();
    void clear_object_list();

    double generate_random_double(double minValue, double maxValue);

};

#endif // PHYSICSSPACE_H