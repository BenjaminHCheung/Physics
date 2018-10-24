#ifndef VECTOR3D_H
#define VECTOR3D_H


class Vector3d
{
public:
    Vector3d(double xInput, double yInput, double zInput);
    ~Vector3d();

    void set_x_value(double xInput);
    void set_y_value(double yInput);
    void set_z_value(double zInput);
    double get_x_value();
    double get_y_value();
    double get_z_value();

    bool operator==(Vector3d &rightHandVector);
    Vector3d operator+(Vector3d &rightHandVector);
    Vector3d operator-(Vector3d &rightHandVector);
    Vector3d operator*(Vector3d &rightHandVector);
    Vector3d operator*(double scaler);
    Vector3d operator/(double divider);
    Vector3d operator/(Vector3d &rightHandVector);

private:
    double mXValue;
    double mYValue;
    double mZValue;


};

#endif // VECTOR3D_H
