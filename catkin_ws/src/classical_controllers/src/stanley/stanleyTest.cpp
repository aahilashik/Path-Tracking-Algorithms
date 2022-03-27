#include <iostream>
#include <boost/circular_buffer.hpp>
#include "stanley_ctrl.cpp"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace std;

string filePath     = "/home/ubuntussd/catkin_ws/src/classical_controllers/data/trajectory_points.txt";
auto isPathExists = [](string path) {
  struct stat buffer;
  return (stat (path.c_str(), &buffer) == 0);
};
vector<vector<double>> loadPointsFromFile() {
  vector<vector<double>> Points;
  if (isPathExists(filePath))
    cout << "INFO : Saved points file exists!" << endl;
  else {
    cout << "INFO : Saved points file does not exists!" << endl;
    // Return empty vector points
    return Points;
  }

  ifstream inFile;
  string line;
  inFile.open(filePath);

  double pX, pY;
  while (std::getline(inFile, line)) {
    istringstream dataStream(line);

    dataStream >> pX;
    dataStream >> pY;

    Points.push_back(vector<double>{pX, pY});
    if ((pX==0) & (pY==15))     break;
  }
  return Points;
}

// Parameters - Stanley Controller
double kE   = 1.0;  // Control Gain
double kS   = 0.0;  // 1e-12; // Softening Constant
// Parameters - PID Controller
double kP   = 1.0;  // Proportional Gain
double kI   = 0.0;  // Integral Gain
double kD   = 0.0;  // Derivative Gain
// Parameters - Vehicle
double x    = 1.0;  // Initial Position
double y    = -15.0;
double yaw  = 0.0;
double vel  = 0.04;  // Initial Target Velocity
double maxSpeed = 1.2;      // m/s
double maxSteer = M_PI/3.0; // 60 deg

// PID Controller
PID pid = PID(kP, kI, kD);
// Stanley Controller
StanleyController stanley = StanleyController(kE, kS);
// Vehicle Simulation
Vehicle diffDrive(x, y, yaw);
int main() {
    double time = 0.0, maxTime = 60, dt = 0.1;    // seconds

    // Trajectory Points
    vector<vector<double>> path = loadPointsFromFile();
    // Trajectory Points splits to x & y
    vector<double> x, y;
    for (auto p:path) {
        x.push_back(p[0]);
        y.push_back(p[1]);
    }
    // Velocity Profile (Manhattan Distance Based)
    vector<double> targetSpeed; //({vel});
    for (int i=0; i<path.size()-1; i++)
        targetSpeed.push_back(min(maxSpeed, (abs(path[i][0]-path[i+1][0]) + abs(path[i][1]-path[i+1][1] + 0.15)) * 2.2 ));
        targetSpeed.push_back(0.4);

    double acceleration, currVelocity = 0.0;
    StanleyData result(0.0, stanley.getTargetIndex(diffDrive, path));
    // Plot Variables
    boost::circular_buffer<double> poseX(10), poseY(10), currVel(100), targVel(100), timeVel(100);
    while ((time <= maxTime) & (result.targetIdx < path.size()-1)) {
        result = stanley.stanleyControl(diffDrive, path, result.targetIdx);
        acceleration = pid.compute(targetSpeed[result.targetIdx], diffDrive.linVel, dt);

        currVelocity += acceleration*dt;
        diffDrive.updateVelocity(currVelocity, currVelocity/result.radiusOrSteer, dt);
        time += dt;

        // cout << path[result.targetIdx][0] << " " << path[result.targetIdx][1] << " ";
        // cout << result.radiusOrSteer << " " << currVelocity << " ";
        // cout << diffDrive.x << " " << diffDrive.y << endl;


        poseX.push_back(diffDrive.x);
        poseY.push_back(diffDrive.y);
        currVel.push_back(currVelocity);
        targVel.push_back(targetSpeed[result.targetIdx]);
        timeVel.push_back(time);


        plt::clf();
        plt::title("Stanley : Path Tracking");  // Add Graph Title

        plt::named_plot("Trajectory", x, y, "ob-");
        // plt::named_plot("Velocity Profile", x, targetSpeed, "orange");

        plt::named_plot("Position", vector<double>(poseX.begin(), poseX.end()), 
                                        vector<double>(poseY.begin(), poseY.end()), "red");
        
        plt::legend();      // Enable legend
        plt::pause(0.01);   // Display plot continuously
    }

    return 1;
}