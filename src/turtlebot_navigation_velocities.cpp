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
		double x_proj = it->x + it->vx * v / 2 / 0.5; // acceleration = 0.5
		double y_proj = it->y + it->vy * v / 2 / 0.5; // acceleration = 0.5

		std::cout << "Proj target x: " << x_proj << ", y: " << y_proj << std::endl;

		// send command
		goal.target_pose.header.frame_id = "/map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = x_proj;
		goal.target_pose.pose.position.y = y_proj;
		tf::quaternionTFToMsg( tf::createQuaternionFromYaw( atan2( it->vy, it->vx ) ), 
							   goal.target_pose.pose.orientation );

		ROS_INFO("Sending goal");
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
			if( fabs( transform.getOrigin().x() - it->x ) < 0.15 && fabs( transform.getOrigin().y() - it->y ) < 0.15 )
				done = true;
		}
		
		// cancel goal and prepare to execute next goal
		ac.cancelGoal();
	}

	return 0;
}
