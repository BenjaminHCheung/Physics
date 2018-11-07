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
    void set_gravity(double xValue, double yValue, double zValue);
    void set_fluid_density(double density);
    void set_max_radius(double maxRadius);
    void set_min_radius(double minRadius);
    void set_max_mass(double maxMass);
    void set_min_mass(double minMass);
    void set_max_Cr(double maxCr);
    void set_min_Cr(double minCr);
    void set_velocity_max(double maxVelocity);
    void set_velocity_min(double minVelocity);
    void update_after_change();

    void timestep_object_list();
    void update_object(SphereObject* physicsObject);
    void object_acceleration_update(SphereObject* physicsObject);
    Vector3d object_drag(SphereObject* physicsObject);
    void object_velocity_update(SphereObject* physicsObject);
    void object_Position_update(SphereObject* physicsObject);

    void check_for_collision(SphereObject* physicsObject);
    bool object_wall_collision(double position, double radius);
    double correct_overshoot(double radius, double oustidePosition);
    bool wrong_direction_velocity(double position, double velocity);
    void fix_velocity_for_bounce(Vector3d bounce, SphereObject* physicsObject);

    void check_object_collisions();
    void do_objects_collide(SphereObject* firstObject, SphereObject* secondObject);
    void two_object_collision(SphereObject* firstObject, SphereObject* secondObject, double magnitudeOfPosition);
    void two_object_reposition(SphereObject* firstObject, SphereObject* secondObject, double combinedRadius, double magnitudeOfPosition);
    double calculate_magnitude_of_position_vectors(SphereObject* firstObject, SphereObject* secondObject);
    Vector3d calculate_new_velocity(SphereObject* firstObject, SphereObject* secondObject, double magnitudeOfPosition);

    std::vector<SphereObject*> get_object_list();
    unsigned int get_objectlist_size();
    void create_physics_objects();

private:
    unsigned int mNumberOfObjects{10};
    unsigned int mOldNumberOfObjects{10};
    unsigned int mNumberOfIterations{3};
    double mTimeStep{1.0/90.0};
    double mRadiusMin{0.2};
    double mRadiusMax{1.0};
    double mRadiusSafetyMargin{.01};
    double mMassMin{2.0};
    double mMassMax{10.0};
    double mCrMin{0.7};
    double mCrMax{1.0};
    double mBoxDimension{5.0};
    double mFluidDensity{1.275};
    double mVelocityMin{0.0};
    double mVelocityMax{6.0};
    Vector3d mGravity{Vector3d(0.0,0.0,-10.0)};
    std::vector<SphereObject*> mObjectList;

    Vector3d generate_random_position();
    Vector3d generate_random_velocity();
    double* generate_random_color();
    void clear_object_list();

    double generate_random_double(double minValue, double maxValue);
};

#endif // PHYSICSSPACE_H
