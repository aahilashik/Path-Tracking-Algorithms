#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <iostream>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Twist.h>

#include "pursuit_ctrl.cpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// Parameters - Pure Pursuit Controller
double klD  = 0.1;  // Look Ahead Gain
double lD   = 1.0;  // Look Ahead Distance
// Parameters - PID Controller
double kP   = 0.8;  // Proportional Gain
double kI   = 0.0;  // Proportional Gain
double kD   = 0.0;  // Proportional Gain
// Parameters - Vehicle
double x    = -3.0;  // Initial Position
double y    = -15.0;
double yaw  = 0.0;
double vel  = 0.1;  // Initial Target Velocity
double maxSpeed = 1.2; // m/s
vector<double> goalPose{-2, -15};   // Goal Pose
double goalTolerance = 0.2;

// PID Controller
PID pid = PID(kP, kI, kD);
// Pure Pursuit Controller
PurePursuit pursuit = PurePursuit(klD, lD);
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

string folderPath   = "~/catkin_ws/src/classical_controllers/data";
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

    ros::init(argc, argv, "pure_pursuit");

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
    // vector<vector<double>> path{{-24.5, -14.5}, {-26, -14}, {-27, -13.5}, {-28, -13}, {-28.5, -12.5}, {-29.2, -12}, {-30, -11.5}, {-30.4, -11}, {-30.8, -10.5}, {-31.2, -10}, {-31.6, -9.5}, {-31.9, -9}, {-32.2, -8.5}, {-32.6, -8}, {-32.85, -7.5}, {-33.0, -7}, {-33.2, -6.5}, {-33.45, -6}, {-33.6, -5.5}, {-33.75, -5}, {-33.9, -4.5}, {-33.95, -4}, {-34, -3.5}, {-34.1, -3}, {-34.15, -2.5}, {-34.2, -2}, {-34.2, -1.5}, {-34.25, -1}, {-34.25, -0.5}, {-34.25, 0}, {-34.25, 0.5}, {-34.25, 1}, {-34.2, 1.5}, {-34.2, 2}, {-34.15, 2.5}, {-34.1, 3}, {-34, 3.5}, {-33.95, 4}, {-33.9, 4.5}, {-33.75, 5}, {-33.6, 5.5}, {-33.45, 6}, {-33.2, 6.5}, {-33.0, 7}, {-32.85, 7.5}, {-32.6, 8}, {-32.2, 8.5}, {-31.9, 9}, {-31.6, 9.5}, {-31.2, 10}, {-30.8, 10.5}, {-30.4, 11}, {-30, 11.5}, {-29.2, 12}, {-28.5, 12.5}, {-28, 13}, {-27, 13.5}, {-26, 14}, {-24.5, 14.5}, {-20, 15}, {-19, 15}, {-18, 15}, {-17, 15}, {-16, 15}, {-15, 15}, {-14, 15}, {-13, 15}, {-12, 15}, {-11, 15}, {-10, 15}, {-9, 15}, {-8, 15}, {-7, 15}, {-6, 15}, {-5, 15}, {-4, 15}, {-3, 15}, {-2, 15}, {-1, 15}, {0, 15}, {1, 15}, {2, 15}, {3, 15}, {4, 15}, {5, 15}, {6, 15}, {7, 15}, {8, 15}, {9, 15}, {10, 15}, {11, 15}, {12, 15}, {15.95, 14.5}, {17.45, 14}, {18.45, 13.5}, {19.45, 13}, {19.95, 12.5}, {20.65, 12}, {21.45, 11.5}, {21.85, 11}, {22.25, 10.5}, {22.65, 10}, {23.05, 9.5}, {23.35, 9}, {23.65, 8.5}, {24.05, 8}, {24.3, 7.5}, {24.45, 7}, {24.65, 6.5}, {24.9, 6}, {25.05, 5.5}, {25.2, 5}, {25.35, 4.5}, {25.4, 4}, {25.45, 3.5}, {25.55, 3}, {25.6, 2.5}, {25.65, 2}, {25.65, 1.5}, {25.7, 1}, {25.7, 0.5}, {25.7, 0}, {25.7, -0.5}, {25.7, -1}, {25.65, -1.5}, {25.65, -2}, {25.6, -2.5}, {25.55, -3}, {25.45, -3.5}, {25.4, -4}, {25.35, -4.5}, {25.2, -5}, {25.05, -5.5}, {24.9, -6}, {24.65, -6.5}, {24.45, -7}, {24.3, -7.5}, {24.05, -8}, {23.65, -8.5}, {23.35, -9}, {23.05, -9.5}, {22.65, -10}, {22.25, -10.5}, {21.85, -11}, {21.45, -11.5}, {20.65, -12}, {19.95, -12.5}, {19.45, -13}, {18.45, -13.5}, {17.45, -14}, {15.95, -14.5}, {12, -15}, {11, -15}, {10, -15}, {9, -15}, {8, -15}, {7, -15}, {6, -15}, {5, -15}, {4, -15}, {3, -15}, {2, -15}, {1, -15}, {0, -15}, {-1, -15}, {-2, -15}, {-3, -15}, {-4, -15}, {-5, -15}, {-6, -15}, {-7, -15}, {-8, -15}, {-9, -15}, {-10, -15}, {-11, -15}, {-12, -15}, {-13, -15}, {-14, -15}, {-15, -15}, {-16, -15}, {-17, -15}, {-18, -15}, {-19, -15}, {-20, -15}};
    // vector<vector<double>> path{{-2, -15}, {-3, -15}, {-4, -15}, {-5, -15}, {-6, -15}, {-7, -15}, {-8, -15}, {-9, -15}, {-10, -15}, {-11, -15}, {-12, -15}, {-13, -15}, {-14, -15}, {-15, -15}, {-16, -15}, {-17, -15}, {-18, -15}, {-19, -15}, {-20, -15}, {-24.5, -14.5}, {-26, -14}, {-27, -13.5}, {-28, -13}, {-28.5, -12.5}, {-29.2, -12}, {-30, -11.5}, {-30.4, -11}, {-30.8, -10.5}, {-31.2, -10}, {-31.6, -9.5}, {-31.9, -9}, {-32.2, -8.5}, {-32.6, -8}, {-32.85, -7.5}, {-33.0, -7}, {-33.2, -6.5}, {-33.45, -6}, {-33.6, -5.5}, {-33.75, -5}, {-33.9, -4.5}, {-33.95, -4}, {-34, -3.5}, {-34.1, -3}, {-34.15, -2.5}, {-34.2, -2}, {-34.2, -1.5}, {-34.25, -1}, {-34.25, -0.5}, {-34.25, 0}, {-34.25, 0.5}, {-34.25, 1}, {-34.2, 1.5}, {-34.2, 2}, {-34.15, 2.5}, {-34.1, 3}, {-34, 3.5}, {-33.95, 4}, {-33.9, 4.5}, {-33.75, 5}, {-33.6, 5.5}, {-33.45, 6}, {-33.2, 6.5}, {-33.0, 7}, {-32.85, 7.5}, {-32.6, 8}, {-32.2, 8.5}, {-31.9, 9}, {-31.6, 9.5}, {-31.2, 10}, {-30.8, 10.5}, {-30.4, 11}, {-30, 11.5}, {-29.2, 12}, {-28.5, 12.5}, {-28, 13}, {-27, 13.5}, {-26, 14}, {-24.5, 14.5}, {-20, 15}, {-19, 15}, {-18, 15}, {-17, 15}, {-16, 15}, {-15, 15}, {-14, 15}, {-13, 15}, {-12, 15}, {-11, 15}, {-10, 15}, {-9, 15}, {-8, 15}, {-7, 15}, {-6, 15}, {-5, 15}, {-4, 15}, {-3, 15}, {-2, 15}, {-1, 15}, {0, 15}, {1, 15}, {2, 15}, {3, 15}, {4, 15}, {5, 15}, {6, 15}, {7, 15}, {8, 15}, {9, 15}, {10, 15}, {11, 15}, {12, 15}, {15.95, 14.5}, {17.45, 14}, {18.45, 13.5}, {19.45, 13}, {19.95, 12.5}, {20.65, 12}, {21.45, 11.5}, {21.85, 11}, {22.25, 10.5}, {22.65, 10}, {23.05, 9.5}, {23.35, 9}, {23.65, 8.5}, {24.05, 8}, {24.3, 7.5}, {24.45, 7}, {24.65, 6.5}, {24.9, 6}, {25.05, 5.5}, {25.2, 5}, {25.35, 4.5}, {25.4, 4}, {25.45, 3.5}, {25.55, 3}, {25.6, 2.5}, {25.65, 2}, {25.65, 1.5}, {25.7, 1}, {25.7, 0.5}, {25.7, 0}, {25.7, -0.5}, {25.7, -1}, {25.65, -1.5}, {25.65, -2}, {25.6, -2.5}, {25.55, -3}, {25.45, -3.5}, {25.4, -4}, {25.35, -4.5}, {25.2, -5}, {25.05, -5.5}, {24.9, -6}, {24.65, -6.5}, {24.45, -7}, {24.3, -7.5}, {24.05, -8}, {23.65, -8.5}, {23.35, -9}, {23.05, -9.5}, {22.65, -10}, {22.25, -10.5}, {21.85, -11}, {21.45, -11.5}, {20.65, -12}, {19.95, -12.5}, {19.45, -13}, {18.45, -13.5}, {17.45, -14}, {15.95, -14.5}, {12, -15}, {11, -15}, {10, -15}, {9, -15}, {8, -15}, {7, -15}, {6, -15}, {5, -15}, {4, -15}, {3, -15}, {2, -15}, {1, -15}, {0, -15}};
    // std::reverse(path.begin(), path.end());
    // savePointsToFile(path);
    // path = loadPointsFromFile();
    vector<vector<double>> path = loadPointsFromFile();
    // cout << path2[2][0] << " " << path2[2][1] << endl;
    // vector<vector<double>> path;
    // for (double i=0; i<20; i+=0.4)
    //     path.push_back({i, sin(i / 3.0) * i / 4.0});

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
        // targetSpeed.push_back(1.6);
    targetSpeed.pop_back();
    targetSpeed.push_back(0.3);

    auto isGoalReached = [](Vehicle veh, vector<double>p2, double tolerance) {
        if (sqrtf(pow(veh.x-p2[0], 2) + pow(veh.y-p2[1], 2)) <= tolerance)
            return true;
        return false;
    };

    vector<double> positionX, positionY, velocity, veloTime;
    double acceleration, currVelocity = 0.0;
    PursuitData result(0.0, pursuit.getTargetIndex(diffDrive, path));
    while ((time <= maxTime) & (result.targetIdx <= path.size()-1) && ros::ok()) {
        if (isGoalReached(diffDrive, goalPose, goalTolerance)) {
            cout << "Goal Reached..!" << endl;
            break;
        }

        result = pursuit.purePursuitControl(diffDrive, path, result.targetIdx);
        acceleration = pid.compute(targetSpeed[result.targetIdx], diffDrive.linVel, dt);

        currVelocity = diffDrive.linVel + acceleration*dt;
        currVelocity = min(1.6, diffDrive.linVel + acceleration*dt);
        // diffDrive.updateVelocity(currVelocity, currVelocity/result.radiusOrSteer, dt);

        // cout << path[min(int(path.size()-1), int(time/dt))][0] << " " << path[min(int(path.size()-1), int(time/dt))][1] << " ";
        // cout << path[result.targetIdx][0] << " " << path[result.targetIdx][1] << " ";
        // cout << result.radiusOrSteer << " " << currVelocity << " ";
        // cout << diffDrive.x << " " << diffDrive.y << endl;
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
        plt::title("Pure Pursuit : Path Tracking");

        plt::named_plot("Trajectory", x, y, "ob-");
        plt::named_plot("Velocity Profile", x, targetSpeed, "orange");

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





    // ros::spin();

    return 0;
}
