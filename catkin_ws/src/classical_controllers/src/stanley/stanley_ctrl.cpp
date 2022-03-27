#include <iostream>
#include <cmath>
#include <bits/stdc++.h>
#include <boost/circular_buffer.hpp>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace std; 

// PID Controller //
class PID {
    public:
        double Kp, Ki, Kd;

        double prevError;
        double _integral, _derivative;
    PID(double kP, double kI=0.0, double kD=0.0) {
        Kp = kP;
        Ki = kI;
        Kd = kD;
        
        prevError    = 0.0;
        _integral    = 0.0;
        _derivative  = 0.0;
    }

    double compute(double target, double current, double dt=0.1) {
        double currError = target - current;

        _derivative      = currError - this->prevError;
        this->_integral += currError;
        
        this->prevError = currError;
        double control  = ( this->Kp * currError ) + 
                                ( this->Ki * this->_integral * dt ) + 
                                    ( this->Kd * this->_derivative / dt );
        return control;
    }
};


// Vehicle Kinematics //
enum VehicleType { Ackermann, Differential };

class Vehicle {
    public:
        double x, y; // Position
        double yaw, maxSteer;  // Orientation
        double linVel, angVel;  // Linear & Angular Velocity

        double length; // Vehicle Dimension
        VehicleType type;

    Vehicle() {
        this->x = this->y = this->yaw   = 0.0;

        this->linVel = this->angVel     = 0.0;

        this->length    = 1.0;
        this->maxSteer  = M_PI/3;   // 60 deg
        this->type      = Differential;
    }

    Vehicle(double x, double y, double yaw, double initLinVel=0.0, double initAngVel=0.0, double bodyLength=1.0, double maxSteer=M_PI/3, VehicleType vehicleType=Differential) {
        this->x     = x;
        this->y     = y;
        this->yaw   = yaw;

        this->linVel    = initLinVel; 
        this->angVel    = initAngVel;

        this->length    = bodyLength;
        this->maxSteer  = maxSteer;
        this->type      = vehicleType;
    }

    void updateVelocity(double linVel, double angVelOrSteer, double dt=0.4) {
        this->x = this->x + (this->linVel * cos(this->yaw) * dt);
        this->y = this->y + (this->linVel * sin(this->yaw) * dt);

        if (this->type == Ackermann)
            this->yaw = this->yaw + (this->linVel / this->length * tan(angVelOrSteer) * dt);    // angVelOrSteer - Steering Angle
        else { // Differential
            this->yaw = this->yaw + (this->angVel * dt);
            this->angVel = angVelOrSteer; // angVelOrSteer - Angular Velocity
        }
        this->linVel = linVel;    // linVel - Linear Velocity
    }

    void updateAcceleration(double linAcc, double angAccOrSteer, double dt=0.4) {
        this->x = this->x + (this->linVel * cos(this->yaw) * dt);
        this->y = this->y + (this->linVel * sin(this->yaw) * dt);

        if (this->type == Ackermann)
            this->yaw = this->yaw + (this->linVel / this->length * tan(angAccOrSteer) * dt);    // angAccOrSteer - Steering Angle
        else { // Differential
            this->yaw = this->yaw + (this->angVel * dt);
            this->angVel = this->angVel + (angAccOrSteer * dt); // angAccOrSteer - Angular Acceleration
        }
        this->linVel = this->linVel + (linAcc * dt);    // linAcc - Linear Acceleration 
    }
};


// Stanley Output Data Type //
struct StanleyData {
    double  radiusOrSteer;  // Radius of the Curve or Steer Angle
    int     targetIdx;      // Target Index of the Trajectory Points
    StanleyData(double rS, int tIdx) : radiusOrSteer(rS), targetIdx(tIdx) {}
};
// Stanley Controller //
class StanleyController {
    public:
        double kE;  // Control Gain
        double kS;  // Softening Constant

        // Constructors
        StanleyController() : kE(1.0), kS(1e-9) {}
        StanleyController(double kE, double kS = 1e-9) : kE(kE), kS(kS) {}

        int getTargetIndex(Vehicle vehicle, vector<vector<double>> points) {
            vector<double> dist;    // Distance between Vehicle and Trajectory Points
            double frontAxleX = vehicle.x + vehicle.length*cos(vehicle.yaw);
            double frontAxleY = vehicle.y + vehicle.length*sin(vehicle.yaw);
            for (auto point:points)
                // Distance = sqrt( x^2 + y^2 )
                dist.push_back(sqrtf(pow(frontAxleX - point[0], 2) + pow(frontAxleY - point[1], 2)));

            vector<double>::iterator it = find(dist.begin(), dist.end(), *min_element(dist.begin(), dist.end()));

            int index = distance(dist.begin(), it);

            return index;
        }

        double calculateEFA(Vehicle vehicle, vector<vector<double>> points, int targetIdx) {
            // Reference : https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
            // Line defined by Point and Angle
            // /*
            double pX = vehicle.x;
            double pY = vehicle.y;
            double pTheta = vehicle.yaw;

            double x0 = points[targetIdx][0];
            double y0 = points[targetIdx][1];

            double errorFrontAxle = cos(pTheta)*(pY - y0) - sin(pTheta)*(pX - x0);
            return -errorFrontAxle;
            // */

            /*
            // Line defined by Two Points
            double x0 = points[targetIdx][0];
            double y0 = points[targetIdx][1];
            // Point1(P1)
            double x1 = vehicle.x;
            double y1 = vehicle.y;
            // Point2(P2)
            double x2 = vehicle.x + vehicle.length*cos(vehicle.yaw);
            double y2 = vehicle.y + vehicle.length*sin(vehicle.yaw);
            
            double errorFrontAxle = ( (x2-x1)*(y1-y0) - (x1-x0)*(y2-y1) ) / sqrtf(pow(x2-x1, 2) + pow(y2-y1, 2));
            return -errorFrontAxle;
            */
        }

        double normalizeAngle(double angle) {
            while (angle > M_PI)    angle -= 2.0*M_PI;
            while (angle < -M_PI)   angle += 2.0*M_PI;
            return angle;
        }

        StanleyData stanleyControl(Vehicle vehicle, vector<vector<double>> points, int targetIdx) {
            int index = max(targetIdx, getTargetIndex(vehicle, points));

            double trajectYaw = ((index != 0) ? atan2(points[index][1]-points[index-1][1], points[index][0]-points[index-1][0]) : 
                                                    atan2(points[index+1][1]-points[index][1], points[index+1][0]-points[index][0]) );

            double efa = calculateEFA(vehicle, points, index);

            // Heading Error
            double thetaE = normalizeAngle(trajectYaw - vehicle.yaw);
            // Cross Track Error
            double thetaD = atan2(this->kE * efa, this->kS + vehicle.linVel);

            double delta = thetaE + thetaD;

            delta = max(-vehicle.maxSteer, min(delta, vehicle.maxSteer));

            if (vehicle.type == Differential) {
                double radius = vehicle.length / tan(delta);
                return StanleyData(radius, index);
            } else if (vehicle.type == Ackermann)
                return StanleyData(delta, index);
        }
};