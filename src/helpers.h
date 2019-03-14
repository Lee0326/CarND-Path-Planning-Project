#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}
// return all the possible states 

vector <string> successor_states(string state, int lane, int max_lane_num) {
  //provide the possible next states given the current state. 
  vector <string> states;
  states.push_back("KL");
  if ((lane > 0) && (lane <(max_lane_num-1))) {
    if (state.compare("KL") == 0) {
      states.push_back("PLCL");
      states.push_back("PLCR");
      
    } else if (state.compare("PLCL") == 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    } else if (state.compare("PLCR") == 0) {
    states.push_back("PLCR");
    states.push_back("LCR");
    }
  } else if (lane == 0) {
    if (state.compare("KL") == 0) {
      states.push_back("PLCR");
    } else if (state.compare("PLCR") == 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  } else if (lane == (max_lane_num-1)) {
    if (state.compare("KL") == 0) {
      states.push_back("PLCL");
    } else if (state.compare("PLCL") == 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }    
  }  
  return states;
}

// calculate the kinematic given the state, current_pose and the sensor fusion data
vector <double> generate_trajectory(double car_speed, double cur_s, string state, double max_speed, int lane, int prev_size, const vector <vector <double>> &predictions) {
  double delta_lane = 0.0;
  double intended_lane = 0.0;
  double delta_s;
  vector <double> vehicle_trajectory = {max_speed,max_speed,max_speed,delta_lane,intended_lane,1000.};  
  // target_speed, intended_speed, final_speed and delta, intended lane and the distance to front car in sequence 
  for (int i=0; i<predictions.size(); i++) {   
    float d = predictions[i][6];
    double vx = predictions[i][3];
    double vy = predictions[i][4];
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = predictions[i][5];    
    check_car_s += (double(prev_size)*.02*check_speed); 
    if (state.compare("KL")==0) {
      // we only care about the front lane in "KL" mode, if there are cars in front, its data would define the kinematic values of this state
      if ((d < (2+4*lane+2)) && (d > (2+4*lane-2))) {
	if ((check_car_s >cur_s) && ((check_car_s-cur_s)<35)) {
	  vehicle_trajectory[1] = check_speed; //set the original lane speed as the intended_speed
	  vehicle_trajectory[2] = check_speed;
	  delta_s = check_car_s - cur_s;
	  vehicle_trajectory[5] = abs(delta_s);
	  
	}//set the original lane speed as the final speed
	delta_lane = 0.;
	intended_lane = 0.;
	vehicle_trajectory[3] = delta_lane;
	vehicle_trajectory[4] = intended_lane;
      }
  }else if (state.compare("PLCL")==0) {    
    if ((d < (2+4*lane+2)) && (d > (2+4*lane-2))) {
      if ((check_car_s-cur_s> 0) && ((check_car_s-cur_s)<35)) {
	  vehicle_trajectory[2] = check_speed;	//set the original lane speed as the final speed
          delta_s = check_car_s - cur_s;
          vehicle_trajectory[5] = abs(delta_s);
	}      
      }else if ((d < (2+4*(lane-1)+2)) && (d > (2+4*(lane-1)-2))) {
	// if there is a vehicle in the intended lane, the intended speed will be the vehicle speed in that lane, else it would be the max_speed
	if ((check_car_s-cur_s> 0) && ((check_car_s-cur_s)<35)) {
	  delta_s = check_car_s - cur_s;
	  if (delta_s > (check_car_s-cur_s)) {
	    vehicle_trajectory[5] = abs(delta_s);
	  } 
	  vehicle_trajectory[1] = check_speed;  
	}
      }
    delta_lane = 0.0;
    intended_lane = -1.0;
    vehicle_trajectory[3] = delta_lane;
    vehicle_trajectory[4] = intended_lane;
    } else if (state.compare("PLCR")==0) {
      if ((d < (2+4*lane+2)) && (d > (2+4*lane-2))) {
	if ((check_car_s >cur_s) && ((check_car_s-cur_s)<35)) {
	  vehicle_trajectory[2] = check_speed;	//set the original lane speed as the final speed
          delta_s = check_car_s - cur_s;
          vehicle_trajectory[5] = abs(delta_s);      
	}      
      }else if ((d < (2+4*(lane+1)+2)) && (d > (2+4*(lane+1)-2))) {
	// if there is a vehicle in the intended lane, the intended speed will be the vehicle speed in that lane, else it would be the max_speed
	if ((check_car_s >cur_s) && ((check_car_s-cur_s)<35)) {
	  delta_s = check_car_s - cur_s;
	  if (delta_s > (check_car_s-cur_s)) {
	    vehicle_trajectory[5] = abs(delta_s);
	  }	  
	  vehicle_trajectory[1] = check_speed; 
	}
      }
    delta_lane = 0.0;
    intended_lane = 1.0;
    vehicle_trajectory[3] = delta_lane;
    vehicle_trajectory[4] = intended_lane;
  } else if (state.compare("LCL")==0) {
    // fot lane change state (LCL or LCR), the final or intended lane are the same, and the cars variables in that lane were used to define state kinematic values. 
      if ((d < (2+4*(lane-1)+2)) && (d > (2+4*(lane-1)-2))) {
	if ((check_car_s-cur_s>=10) && ((check_car_s-cur_s)<35)) {
	  vehicle_trajectory[1] = check_speed;  //set the new lane speed as the intended speed
	  vehicle_trajectory[2] = check_speed;	//set the new lane speed as the final speed  
	}
	if (abs(check_car_s-cur_s)<10) {
          delta_s = check_car_s - cur_s;
          vehicle_trajectory[5] = abs(delta_s);
        } 	  
       }

    delta_lane = -1.0; 
    intended_lane = -1.0;
    vehicle_trajectory[3] = delta_lane;
    vehicle_trajectory[4] = intended_lane;
    if ((check_car_s-cur_s)>0) {
      delta_s = check_car_s - cur_s;
      vehicle_trajectory[5] = abs(delta_s);
    }
   } else if (state.compare("LCR")==0) {
      if ((d < (2+4*(lane+1)+2)) && (d > (2+4*(lane+1)-2))) {
	if ((check_car_s-cur_s>=10) && ((check_car_s-cur_s)<35)) {
	  vehicle_trajectory[1] = check_speed;  //set the new lane speed as the intended speed
	  vehicle_trajectory[2] = check_speed;	//set the new lane speed as the final speed  
	}
	if (abs(check_car_s-cur_s)<10) {
          delta_s = check_car_s - cur_s;
          vehicle_trajectory[5] = abs(delta_s);
        } 	  
       }
    delta_lane = 1.0;
    intended_lane = 1.0;
    vehicle_trajectory[3] = delta_lane;
    vehicle_trajectory[4] = intended_lane;
    }
  }

  return vehicle_trajectory;
}

// calculate the cost given the trajectory (target speed; intended speed and final speed) 
// cost becomes higher for trajectories with intended lane and final lane that have traffic slower than target_speed
double inefficient_cost(const vector <double> &trajectory) {
  double cost = (2.0*trajectory[0] - trajectory[1] - trajectory[2])/(2.0*trajectory[0]);
  return cost;
}

// calculate the cost given the kinematic
// the cost becomes higher when the car is too close to the car in the final lane
double goal_distance_cost(const vector <double> &trajectory) {
  double delta_s = trajectory[5];
  double cost = 1 - exp(-(5 / delta_s));
  return cost;
}


// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];
  double heading = atan2((map_y-y),(map_x-x));
  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

#endif  // HELPERS_H