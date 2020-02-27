#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <fstream>
#include <iostream>
#include <string>
#include <uWS/uWS.h>
#include <vector>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  double ref_vel = 0; //[MpH]
  int lane = 1;
  h.onMessage([&ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          vector<double> ptsx;
          vector<double> ptsy;
          double ref_x{car_x};
          double ref_y{car_y};
          double ref_yaw{deg2rad(car_yaw)};

          //Logic to handle other vehicles in the road
          if (previous_path_x.size() > 0)
          {
            car_s = end_path_s;
          }
          bool too_close{false};
          bool car_on_left_lane{false};

          for (int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6]; //Frenet lateral distance of other obstacles
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy + vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)previous_path_x.size() * 0.02 * check_speed);
            //car in the left lane close to me
            if ((car_d - d) > 3 && abs(check_car_s - car_s) < 30 && d > 0)
            {
              // double vx = sensor_fusion[i][3];
              // double vy = sensor_fusion[i][4];
              // double check_speed = sqrt(vx * vx + vy + vy);
              car_on_left_lane = true;
              // std::cout << "Car " << i << " is on the left lane"
              //           << " at this s: " << check_car_s << std::endl;
              // std::cout << "Ego is at s: " << car_s << std::endl;
            }
            // bool bool1{(car_d - d) > 3};
            // bool bool2{abs(check_car_s - car_s) < 30};
            // bool bool3{d > 0};
            // std::cout << "Car ID == " << i << std::endl;
            // std::cout << "(car_d - d) > 3 == " << bool1 << std::endl;
            // std::cout << "abs(check_car_s - car_s) == " << bool2 << std::endl;
            // std::cout << "d > 0 == " << bool3 << std::endl;

            //car in the right lane close to me
            // if ((car_d - d) > 3.5 && abs(check_car_s - car_s) < 30)
            // {
            //   double vx = sensor_fusion[i][3];
            //   double vy = sensor_fusion[i][4];
            //   double check_speed = sqrt(vx * vx + vy + vy);
            //   car_on_left_lane = true;
            //   std::cout << "Car " << i << " is on the left lane" << std::endl;
            //   std::cout << "Ego is at " << car_s << std::endl;
            // }

            //car in my lane
            if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
            {
              std::cout << "Car " << i << " is on my lane"
                        << " at this s: " << check_car_s << std::endl;
              bool bool1{(check_car_s > car_s)};
              bool bool2{(check_car_s - car_s) < 30};
              if ((check_car_s > car_s) && abs(check_car_s - car_s) < 30)
              {
                too_close = true;
                // std::cout << "Too close? " << too_close << std::endl;
                // if (lane > 0)
                // {
                //   std::cout << "Change lane!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << car_on_left_lane << std::endl;
                //   lane = lane - 1;
                // }
                // if (lane > 0 && (car_on_right_lane == false))
                // {
                //   lane = lane + 1;
                // }
              }
              // std::cout << "(check_car_s > car_s) == " << bool1 << std::endl;
              // std::cout << "(check_car_s - car_s) < 30 == " << bool2 << std::endl;
            }
          }

          if (too_close && lane > 0 && (car_on_left_lane == false) && car_speed > ref_vel - ref_vel * 0.1)
          {
            lane = lane - 1;
          }
          else if (too_close)
          {
            std::cout << "Too close? " << too_close << " and Ego is at s: " << car_s << std::endl;
            ref_vel = ref_vel - 0.224;
          }
          else if (ref_vel < 49.5)
          {
            ref_vel = ref_vel + 0.224;
          }

          //Here I want to find the last couple of points that the car was following to genereate a smooth transition
          //First initialization step
          if (previous_path_x.size() < 2)
          {
            double prev_car_x{car_x - cos(car_yaw)};
            double prev_car_y{car_y - sin(car_yaw)};

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          //All other cases
          else
          {
            ref_x = previous_path_x[previous_path_x.size() - 1];
            ref_y = previous_path_y[previous_path_y.size() - 1];

            double ref_x_prev{previous_path_x[previous_path_x.size() - 2]};
            double ref_y_prev{previous_path_y[previous_path_y.size() - 2]};
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          //Now I create the real new trajectory that the car has to follow
          //3 points equally spaced of 30 meters are created

          vector<double> next_wp0{getXY(car_s + 30, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)};
          vector<double> next_wp1{getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)};
          vector<double> next_wp2{getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y)};

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //Here I shift the car reference angle to zero
          for (int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          //Create the spline that we will use to interpolate those points
          tk::spline s;

          s.set_points(ptsx, ptsy);

          //First, we want to feed the next_vals vectors with the part of the path
          //that was not sent to the simulator during the previous iteration
          for (int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //Now we break the spline into closer points, to specify the speed we want to travel
          double target_x{30};
          double target_y{s(target_x)};
          //Linearization to find the distance from 0 to 30
          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

          double x_add_on = 0;

          //Fill the rest of the planner
          for (int i = 0; i <= 50 - previous_path_x.size(); i++)
          {
            double N = (target_dist / (0.02 * ref_vel / 2.24));
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //Rotation back to the previous reference system
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" ifprevious_path_x.size()
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}