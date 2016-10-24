#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub;

void callback(sensor_msgs::Joy j){
	geometry_msgs::Twist t;
	t.angular.x = 0.0;
	t.angular.y = 0.0;
	t.angular.z = 0.0;
	t.linear.x = 0.0;
	t.linear.y = 0.0;
	t.linear.z = 0.0;
// 		if(j.axes[1] == 1.0)
// 			t.linear.x = 1.0;
// 		if(j.axes[1] == -1.0)
// 			t.linear.x = -1.0;
// 		
// 		if(j.axes[0] == 1.0)
// 			t.angular.z = 1.0;
// 		
// 		if(j.axes[0] == -1.0)
// 			t.angular.z = -1.0;
	t.linear.x = j.axes[1] * (j.axes[2] + 1.01); //.01 pour eviter de multiplier par 0
	t.angular.z = j.axes[0];
	
	
	
	pub.publish(t);
	
}


int main(int argc, char **argv){
	ros::init (argc, argv, "teleop");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/joy",1,callback);
	pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	
	ros::spin();
	return 0;
}
	
	