
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>

#include <codecvt>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

ros::Publisher pointState;
int paly_state = 0;
void state_callback(const move_base_msgs::MoveBaseActionResult& msg)
{	
	std::cout<<"status is "<< msg.status.status << std::endl;
	if(msg.status.status == 3)
	{
		
		std::cout<<"the goal was achieved successfully!"<<std::endl;
		
		std_msgs::String msg;

		std::stringstream ss;
		ss << "1";
		msg.data = ss.str();
		if(paly_state == 0)
		{
			paly_state = 1;
			pointState.publish(msg);
		}
		
		
	}
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "nav_result_callback");
	ros::NodeHandle n;

	ros::Subscriber goal_sub =n.subscribe("move_base/result", 10, state_callback);
	pointState = n.advertise<std_msgs::String>("/get_pos", 10);
	
	
	ros::spin();
	return 0;
}
