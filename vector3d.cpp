#include "vector3d.h"

Vector3d::Vector3d(double xInput, double yInput, double zInput)
{
    set_x_value(xInput);
    set_y_value(yInput);
    set_z_value(zInput);
}

Vector3d::~Vector3d(){}

void Vector3d::set_x_value(double xInput)
{
    mXValue = xInput;
}

void Vector3d::set_y_value(double yInput)
{
    mYValue = yInput;
}

void Vector3d::set_z_value(double zInput)
{
    mZValue = zInput;
}

double Vector3d::get_x_value()
{
    return mXValue;
}

double Vector3d::get_y_value()
{
    return mYValue;
}

double Vector3d::get_z_value()
{
    return mZValue;
}

bool Vector3d::operator==(Vector3d &rightHandVector)
{
    if(mXValue == rightHandVector.get_x_value() &&
           mYValue == rightHandVector.get_y_value() &&
            mZValue == rightHandVector.get_z_value())
    {
        return true;
    }
    else
    {
        return false;
    }
}

Vector3d Vector3d::operator+(Vector3d &rightHandVector)
{
    double xValue{mXValue + rightHandVector.get_x_value()};
    double yValue{mYValue + rightHandVector.get_y_value()};
    double zValue{mZValue + rightHandVector.get_z_value()};
    return Vector3d(xValue, yValue, zValue);
}

Vector3d Vector3d::operator-(Vector3d &rightHandVector)
{
    double xValue{mXValue - rightHandVector.get_x_value()};
    double yValue{mYValue - rightHandVector.get_y_value()};
    double zValue{mZValue - rightHandVector.get_z_value()};
    return Vector3d(xValue, yValue, zValue);
}

Vector3d Vector3d::operator*(Vector3d &rightHandVector)
{
    double xValue{mXValue*rightHandVector.get_x_value()};
    double yValue{mYValue*rightHandVector.get_y_value()};
    double zValue{mZValue*rightHandVector.get_z_value()};
    return Vector3d(xValue, yValue, zValue);
}

Vector3d Vector3d::operator*(double scaler)
{
    Vector3d newVector{Vector3d(mXValue*scaler, mYValue*scaler, mZValue*scaler)};
    return newVector;
}

Vector3d Vector3d::operator/(double divider)
{
    Vector3d newVector{Vector3d(mXValue/divider, mYValue/divider, mZValue/divider)};
    return newVector;
}

Vector3d Vector3d::operator/(Vector3d &rightHandVector)
{
    double xValue{mXValue/rightHandVector.get_x_value()};
    double yValue{mYValue/rightHandVector.get_y_value()};
    double zValue{mZValue/rightHandVector.get_z_value()};
    return Vector3d(xValue, yValue, zValue);
}
