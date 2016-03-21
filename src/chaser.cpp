#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>

ros::Publisher twsPub;
double xi=4.0, yi= 1.5 , xf = -3.0 , yf = 1.5;
double delta = 0.5;
double ru, theta, th, betha, r, psi, xc, yc;
geometry_msgs::Twist tws;

double normAng (double ang){
	while(ang/M_PI > 1){
		ang = ang - 2*M_PI;
	}
	while(ang/M_PI <= -1){
		ang = ang + 2*M_PI;
	}
	return ang;
}

void twsGet(double yaw){
	theta = std::atan2((yf-yi),(xf-xi));
	betha = normAng(theta - th);
	r = sqrt((ru*ru)-pow(ru*std::sin(betha),2));

	xc = xi+((r+delta)*std::cos(theta));
	yc = yi+((r+delta)*std::sin(theta));
	
	tws.linear.x = 1;
	tws.angular.z = 2.5*normAng(psi-yaw);
}

void callback(const nav_msgs::OdometryConstPtr &od){

	double yaw = tf::getYaw(od->pose.pose.orientation);
	psi = std::atan2((yc-od->pose.pose.position.y),(xc-od->pose.pose.position.x));
	
	ru = sqrt(pow((xi-od->pose.pose.position.x),2)+pow((yi-od->pose.pose.position.y),2));
	th = std::atan2((yi-od->pose.pose.position.y),(xi-od->pose.pose.position.x));
	
	twsGet(yaw);

	twsPub.publish(tws);
	ROS_INFO("published....");
}

int main(int argc, char **argv){
	ros::init(argc,argv,"cChaser");
	ros::NodeHandle chaser;
	ros::Subscriber odSub = chaser.subscribe("/vrep/vehicle/odometry",1, callback);
	twsPub = chaser.advertise <geometry_msgs::Twist>("/twist",1);
	ros::spin();
}
