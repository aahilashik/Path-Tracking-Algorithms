#include <iostream>
#include <cmath>
#include <bits/stdc++.h>

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
        double yaw;  // Orientation
        double linVel, angVel;  // Linear & Angular Velocity

        double length; // Vehicle Dimension
        VehicleType type;

    Vehicle() {
        this->x = this->y = this->yaw   = 0.0;

        this->linVel = this->angVel     = 0.0;

        this->length    = 0.0;
        this->type      = Differential;
    }

    Vehicle(double x, double y, double yaw, double initLinVel=0.0, double initAngVel=0.0, double bodyLength=0.0, VehicleType vehicleType=Differential) {
        this->x     = x;
        this->y     = y;
        this->yaw   = yaw;

        this->linVel    = initLinVel; 
        this->angVel    = initAngVel;

        this->length    = bodyLength;
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


// Pure Pursuit Output Data Type //
struct PursuitData {
    double  radiusOrSteer;  // Radius of the Curve or Steer Angle
    int     targetIdx;      // Target Index of the Trajectory Points
    PursuitData(double r, int t) : radiusOrSteer(r), targetIdx(t) {} //{
    //     radiusOrSteer = r;
    //     targetIdx = t;
    // }
};
// Pure Pursuit Controller(Adaptive) //
class PurePursuit {
    public:
        double lD, klD; // Look Ahead Distance and its Gain

        PurePursuit() : lD(2.0), klD(0.0) {} //{
        //     this->klD = 0.0;
        //     this->lD  = 2.0;
        // }
        PurePursuit(double klD, double lD) : lD(lD), klD(klD) {} //{
        //     this->klD = kld;
        //     this->lD  = ld;
        // }

        int getTargetIndex(Vehicle vehicle, vector<vector<double>> points) {
            vector<double> dist;    // Distance between Vehicle and Trajectory Points
            for (auto point:points) {
                dist.push_back(sqrtf(pow(vehicle.x - point[0], 2) + pow(vehicle.y - point[1], 2)));
            }

            vector<double>::iterator it = find(dist.begin(), dist.end(), *min_element(dist.begin(), dist.end()));

            int index = distance(dist.begin(), it);
            double length = 0.0;
            double newlD = (this->klD * vehicle.linVel) + this->lD; // Adaptive Look Ahead Distance
            while ((newlD > length) & (index < points.size()-1)) {
                double x0 = points[index][0];
                double y0 = points[index][1];
                double x1 = points[index+1][0];
                double y1 = points[index+1][1];
                length += sqrtf(pow(x1 - x0, 2) + pow(y1 - y0, 2));
                index++;
            }
            return index;
        }

        PursuitData purePursuitControl(Vehicle vehicle, vector<vector<double>> points, int targetIdx) {
            int index = max(targetIdx, getTargetIndex(vehicle, points));

            vector<double> target = ( index<points.size() ? points[index] : points.back() );

            double angAlpha = atan2(target[1] - vehicle.y, target[0] - vehicle.x) - vehicle.yaw;

            angAlpha = ( vehicle.linVel<0 ? M_PI - angAlpha : angAlpha );    // If vehicle moves backward

            double newlD = this->klD*vehicle.linVel + this->lD;

            if (vehicle.type == Differential) {
                double radius = newlD / (2.0 * sin(angAlpha) + 1e-12);
                return PursuitData(radius, index);
            } else if (vehicle.type == Ackermann) {
                double delta = atan2(2.0 * vehicle.length * sin(angAlpha), newlD);
                return PursuitData(delta, index);
            }
        }
};

/*
// Parameters - Pure Pursuit Controller
double klD  = 0.6;  // Look Ahead Gain
double lD   = 2.0;  // Look Ahead Distance
// Parameters - PID Controller
double kP   = 4.0;  // Proportional Gain
double kI   = 0.0;  // Proportional Gain
double kD   = 0.1;  // Proportional Gain
// Parameters - Vehicle
double x    = 0.0;  // Initial Position
double y    = 0.0;
double yaw  = 0.0;
double vel  = 1.0;  // Initial Target Velocity
double maxSpeed = 6.0; // m/s

// PID Controller
PID pid = PID(kP, kI, kD);
// Pure Pursuit Controller
PurePursuit pursuit = PurePursuit(klD, lD);
// Vehicle Simulation
Vehicle diffDrive(x, y, yaw);
int main() {
    double time = 0.0, maxTime = 60, dt = 0.3;    // seconds

    // Trajectory Points
    vector<vector<double>> path;
    for (double i=0; i<60; i+=1)
        path.push_back({i, sin(i / 5.0) * i / 1.5});

    // Velocity Profile (Manhattan Distance Based)
    vector<double> targetSpeed({vel});
    for (int i=0; i<path.size()-1; i++)
        targetSpeed.push_back(min(maxSpeed, (abs(path[i][0]-path[i+1][0]) + abs(path[i][1]-path[i+1][1])) * 2.0 ));

    double acceleration, currVelocity = 0.0;
    PursuitData result(0.0, pursuit.getTargetIndex(diffDrive, path));
    while ((time <= maxTime) & (result.targetIdx < path.size()-1)) {
        result = pursuit.purePursuitControl(diffDrive, path, result.targetIdx);
        acceleration = pid.compute(targetSpeed[result.targetIdx], diffDrive.linVel, dt);

        currVelocity += acceleration*dt;
        diffDrive.updateVelocity(currVelocity, currVelocity/result.radiusOrSteer, dt);

        cout << path[min(int(path.size()-1), int(time/dt))][0] << " " << path[min(int(path.size()-1), int(time/dt))][1] << " ";
        cout << diffDrive.x << " " << diffDrive.y << endl;
        time += dt;
    }
  





    // diffDrive.updateAcceleration(2.0, 2.0/4.0);
    // for (int i=0; i<100; i++) {
    //     diffDrive.updateVelocity(2.0, 2.0/4.0);
    //     cout << diffDrive.x << " " << diffDrive.y << endl;
    // }
    return 1;
}  
*/