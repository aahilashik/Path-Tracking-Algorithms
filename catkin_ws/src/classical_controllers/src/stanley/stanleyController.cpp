#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <iostream>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>

#include "stanley_ctrl.cpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
// Parameters - Stanley Controller
double kE   = 1.0;      // Control Gain
double kS   = 1e-12;    // Softening Constant
// Parameters - PID Controller
double kP   = 1.0;  // Proportional Gain
double kI   = 0.0;  // Integral Gain
double kD   = 0.0;  // Derivative Gain
// Parameters - Vehicle
double x    = 0.0;  // Initial Position
double y    = -15.0;
double yaw  = 0.0;
double vel  = 0.01;  // Initial Target Velocity
double maxSpeed = 1.2;      // m/s
double maxSteer = M_PI/3.0; // 60 deg
vector<double> goalPose{-2, -15};   // Goal Pose
double goalTolerance = 0.4;

// PID Controller
PID pid = PID(kP, kI, kD);
// Stanley Controller
StanleyController stanley = StanleyController(kE, kS);
// Vehicle Simulation
Vehicle diffDrive(x, y, yaw);

void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg) {

        tf::Quaternion quat(    msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z,
                                msg->pose.pose.orientation.w    );
        tf::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

    diffDrive.x = msg->pose.pose.position.x;
    diffDrive.y = msg->pose.pose.position.y;

    diffDrive.yaw = yaw;

    diffDrive.linVel = msg->twist.twist.linear.x;
    diffDrive.angVel = msg->twist.twist.angular.z;
}

string filePath     = "/home/ubuntussd/catkin_ws/src/classical_controllers/data/trajectory_points.txt";

auto isPathExists = [](string path) {
  struct stat buffer;
  return (stat (path.c_str(), &buffer) == 0);
};

void savePointsToFile(vector<vector<double>> Points) {
  double pX, pY;
  ostringstream dataStream;
  for (auto point: Points) {
    pX = point[0];
    pY = point[1];

    dataStream << pX << " " << pY << "\n";
  }

  string data = dataStream.str();

  ofstream outFile(filePath);
  outFile << data;
  outFile.close();
}

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
  }
  return Points;
}

int main(int argc, char **argv) {
    double time = 0.0, maxTime = 300, dt = 0.1;    // seconds

    ros::init(argc, argv, "stanley");

    ros::NodeHandle nh;
    ros::Rate rate(1.0/dt);
    ros::Subscriber subOdom = nh.subscribe("/odom", 1, callbackOdom);
    ros::Publisher  pubVel  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    geometry_msgs::Twist velMsg;


    // Delay to update Vehicle Odometry
    cout << "4 sec Delay Begin!" << endl;
    ros::spinOnce();
    ros::Duration(4.0).sleep();
    ros::spinOnce();
    cout << "4 sec Delay Done!" << endl;

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
        targetSpeed.push_back(min(maxSpeed, (abs(path[i][0]-path[i+1][0]) + abs(path[i][1]-path[i+1][1] + 0.15)) * 2.2 ));
    targetSpeed.pop_back();     targetSpeed.pop_back();
    targetSpeed.push_back(0.4); targetSpeed.push_back(0.2);

    auto isGoalReached = [](Vehicle veh, vector<double>p2, double tolerance) {
        if (sqrtf(pow(veh.x-p2[0], 2) + pow(veh.y-p2[1], 2)) <= tolerance)
            return true;
        return false;
    };

    vector<double> positionX, positionY, velocity, veloTime;
    double acceleration, currVelocity = 0.0;
    StanleyData result(0.0, stanley.getTargetIndex(diffDrive, path));
    while ((time <= maxTime) & (result.targetIdx <= path.size()-1) && ros::ok()) {
        if (isGoalReached(diffDrive, goalPose, goalTolerance)) {
            cout << "Goal Reached..!" << endl;
            break;
        }

        result = stanley.stanleyControl(diffDrive, path, result.targetIdx);
        acceleration = pid.compute(targetSpeed[result.targetIdx], diffDrive.linVel, dt);

        currVelocity = diffDrive.linVel + acceleration*dt;

        cout << path[result.targetIdx][0] << " " << path[result.targetIdx][1] << " ";
        cout << result.radiusOrSteer << " " << currVelocity << " ";
        cout << diffDrive.x << " " << diffDrive.y << endl;
        time += dt;


        velMsg.linear.x = currVelocity;
        velMsg.angular.z = round( currVelocity/result.radiusOrSteer * 10000.0 ) / 10000.0;
        pubVel.publish(velMsg);
        ros::spinOnce();

        positionX.push_back(diffDrive.x);
        positionY.push_back(diffDrive.y);
        veloTime.push_back(time/dt * 0.4);
        velocity.push_back(currVelocity);
        // cout << diffDrive.x << " " << diffDrive.y << " " << currVelocity << " " << acceleration << endl;


        plt::clf();

        // Add graph title
        plt::title("Stanley : Path Tracking");

        plt::named_plot("Trajectory", x, y, "ob-");
        // plt::named_plot("Velocity Profile", x, targetSpeed, "orange");

        plt::named_plot("Position", vector<double>(max(positionX.begin(), positionX.end()-10), positionX.end()), 
                                        vector<double>(max(positionY.begin(), positionY.end()-10), positionY.end()), "red");
        // plt::named_plot("Velocity", vector<double>(max(veloTime.begin(), veloTime.end()-100), veloTime.end()), 
        //                                 vector<double>(max(velocity.begin(), velocity.end()-100), velocity.end()), "og-");
        plt::scatter(   vector<double>{path[result.targetIdx][0]}, 
                            vector<double>{path[result.targetIdx][1]}, 2.0, {{"color", "black"}});

        // Enable legend.
        plt::legend();
        // Display plot continuously
        plt::pause(0.01);


        rate.sleep();
    }
    velMsg.linear.x     = 0.0;
    velMsg.angular.z    = 0.0;
    pubVel.publish(velMsg);
    ros::spinOnce();
    rate.sleep();

    return 0;
}
