#include <vector>
#include <fstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct WayPoint
{
	double x;
	double y;
	double vx;
	double vy;

	WayPoint(double x, double y, double vx, double vy) : x(x), y(y), vx(vx), vy(vy) { }
	WayPoint() { }
};

std::vector<WayPoint> points; 

void FillPoints()
{
	std::string cfg_file(getenv("HOME"));
	cfg_file.append( "/catkin_ws/src/turtlebot_navigation_velocities/robo.cfg" );
	std::ifstream fin( cfg_file.c_str(), std::ios::ios_base::in );

	WayPoint pt;
	while(fin >> pt.x >> pt.y >> pt.vx >> pt.vy)
		points.push_back( pt );

	fin.close();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "turtlebot_navigation_velocities");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	tf::TransformListener listener;
	listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));

	tf::StampedTransform transform;
	ros::Rate rate(2.0);

	rate.sleep();
	
	// setup points
	FillPoints();

	for( std::vector<WayPoint>::iterator it = points.begin(); it != points.end(); ++it )
	{
		listener.lookupTransform( "map", "base_link", ros::Time(0), transform );
		
		std::cout << "Robot x: " << transform.getOrigin().x() << ", y: " << transform.getOrigin().y() << std::endl;
		std::cout << "Target x: " << it->x << ", y: " << it->y << std::endl;

		// calculate projected point
		double v = sqrt( pow( it->vx, 2 ) + pow( it->vy, 2 ) );
		double delta_x = it->vx * v / 2 / 0.5; // acceleration = 0.5
		double delta_y = it->vy * v / 2 / 0.5; // acceleration = 0.5

		// acceleration track 0 -> 1
		double x_proj_0 = it->x - delta_x;
		double y_proj_0 = it->y - delta_y;

		double x_proj_1 = it->x + delta_x;
		double y_proj_1 = it->y + delta_y;

		std::cout << "Proj target 0 x: " << x_proj_0 << ", y: " << y_proj_0 << std::endl;
		std::cout << "Proj target 1 x: " << x_proj_1 << ", y: " << y_proj_1 << std::endl;

		// send command
		goal.target_pose.header.frame_id = "/map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = x_proj_0;
		goal.target_pose.pose.position.y = y_proj_0;
		tf::quaternionTFToMsg( tf::createQuaternionFromYaw( atan2( it->vy, it->vx ) ), 
							   goal.target_pose.pose.orientation );

		ROS_INFO("Sending goal to 0 point");
		ac.sendGoal(goal);

		ac.waitForResult();

		if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
			return -1;

		// move to point 1(orientation is the same)
		goal.target_pose.pose.position.x = x_proj_1;
		goal.target_pose.pose.position.y = y_proj_1;

		ROS_INFO("Sending goal to 1 point");
		ac.sendGoal(goal);

		bool done = false;

		rate = 1.0;
		while( !done )
		{
			rate.sleep();

			try
			{
				listener.lookupTransform("map","base_link",ros::Time(0), transform);
			}
			catch (tf::TransformException ex)
			{
				ROS_ERROR("Error transform: %s", ex.what());
			}

			std::cout << "Robot moving x: " << transform.getOrigin().x() << ", y: " << transform.getOrigin().y() << std::endl;
			if( fabs( transform.getOrigin().x() - it->x ) < 0.1 && fabs( transform.getOrigin().y() - it->y ) < 0.1 )
				done = true;
		}
		
		// cancel goal and prepare to execute next goal
		ac.cancelGoal();
	}

	return 0;
}
