#include <ros/ros.h>
#include <flocking/State.h>
#include <unordered_map>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include <turtlesim/Spawn.h>

using Vector2 = Eigen::Matrix<double, 2, 1>;

std::unordered_map<std::string, flocking::State> swarm_state;
flocking::State my_state;

void boid_callback(const flocking::State::ConstPtr& msg) {
  if(msg->id == my_state.id)
    return;
  swarm_state[msg->id] = *msg;
}

void pose_callback(const turtlesim::Pose::ConstPtr& msg) {
  my_state.pos_x = msg->x;
  my_state.pos_y = msg->y;
}

Vector2 velocity_matching() {
  Vector2 suggestion(0, 0);
  int count = 0;
  for(auto it = swarm_state.begin(); it != swarm_state.end(); it++) {
    Vector2 peer_vel(it->second.vel_x, it->second.vel_y);
    suggestion += peer_vel;
    count++;
  }

  if(count != 0) {
    return suggestion / count;
  }
  return suggestion;
}

Vector2 flock_centering() {
  Vector2 suggestion(0, 0);
  int count = 0;
  Vector2 my_pos(my_state.pos_x, my_state.pos_y);
  for(auto it = swarm_state.begin(); it != swarm_state.end(); it++) {
    Vector2 peer_pos(it->second.pos_x, it->second.pos_y);
    suggestion += peer_pos - my_pos;
    count++;
  }

  if(count != 0) {
    return suggestion / count;
  }
  return suggestion;
}

Vector2 collision_avoidance() {
  Vector2 suggestion(0, 0);
  int count = 0;
  Vector2 my_pos(my_state.pos_x, my_state.pos_y);
  for(auto it = swarm_state.begin(); it != swarm_state.end(); it++) {
    Vector2 peer_pos(it->second.pos_x, it->second.pos_y);
    Vector2 seperation = (my_pos - peer_pos);
    double seperation_norm = seperation.norm();
    if(seperation_norm != 0) {
      seperation.normalize();
      suggestion += seperation / seperation_norm;
      count++;
    }
  }
  if(count != 0){
    return suggestion / count;
  }
  return suggestion;
}

Vector2 wall_avoidance() {
  Vector2 suggestion(0, 0);
  Vector2 my_pos(my_state.pos_x, my_state.pos_y);

  if(my_pos(0) != 0) {
    suggestion += Vector2(1, 0) / my_pos(0);
  } else {
    suggestion += Vector2(1, 0) * 1000000;
  }

  if(my_pos(1) != 0) {
    suggestion += Vector2(0, 1) / my_pos(1);
  } else {
    suggestion += Vector2(0, 1) * 1000000;
  }

  if(my_pos(0) != 11) {
    suggestion += Vector2(-1, 0) / std::abs(11 - my_pos(0));
  } else {
    suggestion += Vector2(-1, 0) * 1000000;
  }

  if(my_pos(1) != 11) {
    suggestion += Vector2(0, -1) / std::abs(11 - my_pos(1));
  } else {
    suggestion += Vector2(0, -1) * 1000000;
  }

  return suggestion / 4;
}

Vector2 velocity_preservation() {
  Vector2 cur_vel(my_state.vel_x, my_state.vel_y);
  return cur_vel;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "boid");
  ros::NodeHandle nh;

  my_state.id = ros::this_node::getNamespace();


  if(my_state.id != "/turtle1") {

    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

    turtlesim::Spawn srv;
    srv.request.x = 0;
    srv.request.y = 0;
    srv.request.theta = 0;
    srv.request.name = my_state.id;

    while(!client.call(srv));
  }

  ros::Subscriber boid_sub = nh.subscribe("/boid_states", 1000, boid_callback);
  ros::Subscriber pose_sub = nh.subscribe("pose", 1, pose_callback);
  ros::Publisher des_vel_pub = nh.advertise<geometry_msgs::Twist>("desired_vel", 1);
  ros::Publisher boid_pub = nh.advertise<flocking::State>("/boid_states", 1);

  ros::Rate rate(10);
  double kw = 1.5;
  double kc = 5;
  double kv = 1;
  double kf = 1;
  double kvp = 2;

  double max_vel = 1;
  while(ros::ok()) {
    ros::spinOnce();
    Vector2 suggestion = kv * velocity_matching() 
                       + kf * flock_centering() 
                       + kc * collision_avoidance() 
                       + kw * wall_avoidance() 
                       + kvp * velocity_preservation()
                       ;
    if(suggestion.norm() != 0)
      suggestion.normalize();
    suggestion *= max_vel;

    geometry_msgs::Twist msg;
    msg.linear.x = suggestion(0);
    msg.linear.y = suggestion(1);
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;

    des_vel_pub.publish(msg);


    my_state.vel_x = suggestion(0);
    my_state.vel_y = suggestion(1);

    boid_pub.publish(my_state);


    rate.sleep();
  }
  return 0;
}