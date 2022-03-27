#include <iostream>
#include <boost/circular_buffer.hpp>
#include "pursuit_ctrl.cpp"
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

// Parameters - Pure Pursuit Controller
double klD  = 0.6;  // Look Ahead Gain
double lD   = 2.0;  // Look Ahead Distance
// Parameters - PID Controller
double kP   = 4.0;  // Proportional Gain
double kI   = 0.0;  // Proportional Gain
double kD   = 0.1;  // Proportional Gain
// Parameters - Vehicle
double x    = 0.0;  // Initial Position
double y    = -15.0;
double yaw  = 0.0;
double vel  = 1.0;  // Initial Target Velocity
double maxSpeed = 6.0; // m/s
vector<double> goalPose{0, 15};   // Goal Pose
double goalTolerance = 0.3;

// PID Controller
PID pid = PID(kP, kI, kD);
// Pure Pursuit Controller
PurePursuit pursuit = PurePursuit(klD, lD);
// Vehicle Simulation
Vehicle diffDrive(x, y, yaw);
int main() {
    double time = 0.0, maxTime = 60, dt = 0.1;    // seconds
    plt::figure_size(1280, 720);    // Set Figure Size

    // Trajectory Points
    vector<vector<double>> path = loadPointsFromFile();
    // Trajectory Points splits to x & y
    vector<double> x, y;
    for (auto p:path) {
        x.push_back(p[0]);
        y.push_back(p[1]);
    }
    // Velocity Profile (Manhattan Distance Based)
    vector<double> targetSpeed({vel});
    for (int i=0; i<path.size()-1; i++)
        targetSpeed.push_back(min(maxSpeed, (abs(path[i][0]-path[i+1][0]) + abs(path[i][1]-path[i+1][1] + 0.15)) * 8.0 ));

    auto isGoalReached = [](Vehicle veh, vector<double>p2, double tolerance) {
        if (sqrtf(pow(veh.x-p2[0], 2) + pow(veh.y-p2[1], 2)) <= tolerance)
            return true;
        return false;
    };
    
    double acceleration, currVelocity = 0.0;
    PursuitData result(0.0, pursuit.getTargetIndex(diffDrive, path));
    // Plot Variables
    boost::circular_buffer<double> poseX(10), poseY(10), currVel(100), targVel(100), timeVel(100);
    while ((time <= maxTime) & (result.targetIdx <= path.size()-1)) {
        if (isGoalReached(diffDrive, goalPose, goalTolerance)) {
            cout << "Goal Reached..!" << endl;
            break;
        }
        result = pursuit.purePursuitControl(diffDrive, path, result.targetIdx);
        acceleration = pid.compute(targetSpeed[result.targetIdx], diffDrive.linVel, dt);

        currVelocity += acceleration*dt;
        diffDrive.updateVelocity(currVelocity, currVelocity/result.radiusOrSteer, dt);

        time += dt;

        poseX.push_back(diffDrive.x);
        poseY.push_back(diffDrive.y);
        currVel.push_back(currVelocity);
        targVel.push_back(targetSpeed[result.targetIdx]);
        timeVel.push_back(time);


        plt::clf();
        plt::title("Pure Pursuit : Path Tracking");  // Add Graph Title

        plt::named_plot("Trajectory", x, y, "ob-");
        // plt::named_plot("Velocity Profile", x, targetSpeed, "orange");

        plt::named_plot("Position", vector<double>(poseX.begin(), poseX.end()), 
                                        vector<double>(poseY.begin(), poseY.end()), 
                                            "or-");
        
        plt::legend();      // Enable legend
        plt::pause(0.01);   // Display plot continuously
    }

    return 1;
}