/*
 * rosserial Publicador de ejemplo.
 * Imprime "Ejemplo Arduino - ROS"
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("publicador", &str_msg);

char hello[25] = "Ejemplo Arduino - ROS";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
