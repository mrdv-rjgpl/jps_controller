#include <ros/ros.h>
#include <ur_kinematics/ur_kin.h>
//#include "assignment1.hpp"
#include <Eigen/Eigen>
#include <Eigen/Dense>
// #include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
// #include<geometry_msgs/Pose.h>
#include <geometry_msgs/Pose.h>
// #include<rob_param.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <math.h>
#include<std_msgs/UInt16.h>
#include<std_msgs/Bool.h>


class main_controller{
private:
	ros::NodeHandle nh;
	// ros::Subscriber sub_cameraCal;
	ros::Subscriber sub_puzzlePiece;
	ros::Publisher pub_trajectory;
	ros::Publisher pub_gripper;
	ros::Publisher pub_moved;
	
	tf::TransformListener listener;

	//constants needed
	geometry_msgs::Pose camframe;
	geometry_msgs::Pose home;
	double gripper_offset;
	double table_offset;
	std_msgs::UInt16 angle_grip;
	std_msgs::UInt16 angle_ungrip;
	double z_distance;
	double stepsize;
	std_msgs::Bool msg;
	


	void travelAcrossPlane()
	{
	    ros::Duration(5).sleep();
	    std::cout<<"in travel"<<std::endl;
		geometry_msgs::Pose pose;
		pose.position.x=0.266;
		pose.position.y=0.422;
		pose.position.z=0.432;
		pose.orientation.x=-0.271;
		pose.orientation.y=0.653;
		pose.orientation.z=0.272;
		pose.orientation.w=0.653;
		home=pose;
		double temp=pose.position.y;

		// pose.position=(0.266, 0.422, 0.432);
		// pose.orientation(-0.271, 0.653, 0.272, 0.653);
		// tf::Quaternion quat(-0.271, 0.653, 0.272, 0.653);
		// Eigen::Vector3d pos(0.266, 0.422, 0.432);
		pub_trajectory.publish(pose);
		ros::Duration(10).sleep();
		pub_moved.publish(msg);
		ros::Duration(2).sleep();
		for(int i=1;i<=5;i++)
		{
			for(int j=1;j<=5;j++)
			// pose.position.x+=1/50.0;
			{
				pose.position.y+=1.5/50.0;
				std::cout<<pose<<std::endl;
				pub_trajectory.publish(pose);
				ros::Duration(10).sleep();
				pub_moved.publish(msg);
				ros::Duration(2).sleep();
				std::cout<<"pose executed"<<std::endl;
			}
			pose.position.x-=1.5/50;
			pose.position.y=temp;
			pub_trajectory.publish(pose);
			ros::Duration(10).sleep();	
			pub_moved.publish(msg);
			ros::Duration(2).sleep();		
		}
		std::cout<<"after everything"<<std::endl;

	}

	//get transform between ee and camera. After hand-eye cal. 
	void setCameraPose()
	{
		// camframe=p;
		// camframe.position.x=p.position.x;
	}
	//pose from base to puzzle
	void puzzleSolver(const geometry_msgs::PoseStamped& puzzlePose)
	{
		//frames base2puzzle, base2ee, ee2camera, camera2puzzle. base -> base_link
		Eigen::Matrix<double,4,4> F_bp, F_be, F_ec, F_cp;
		//gripper offset


		F_bp=pose2frame(puzzlePose.pose);
		// F_ec=pose2frame(camframe);
		// tf::StampedTransform base_ee_transform;
		// try
		// {
		// 	listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
		// 	listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);
		// }
		// catch (tf::TransformException ex) {
		// 	ROS_ERROR("Not found");
		// 	ros::Duration(1.0).sleep();
			
		// }
		// //convert from tf::transform to frame. calculate F_bp
		// // geometry_msgs::Pose base2ee = [base_ee_transform.getOrigin(), base_ee_transform.getRotation()];
		// geometry_msgs::Pose base2ee;
		// base2ee.orientation.w = base_ee_transform.getRotation().w();
		// base2ee.orientation.x = base_ee_transform.getRotation().x();
		// base2ee.orientation.y = base_ee_transform.getRotation().y();
		// base2ee.orientation.z = base_ee_transform.getRotation().z();
		
		// base2ee.position.x = base_ee_transform.getOrigin().x();
		// base2ee.position.y = base_ee_transform.getOrigin().y();
		// base2ee.position.z = base_ee_transform.getOrigin().z();
		
		// F_be=pose2frame(base2ee);

		//computes pose from base to puzzle
		// F_bp = F_be * F_ec * F_cp;


		//frame from base to vaccuum gripper
		Eigen::Matrix<double,4,4> F_bvg=F_bp;
		F_bvg(3,3)+=table_offset;

		//this is the goal frame from base to ee. 
		Eigen::Matrix<double,4,4> F_goal=F_bvg;
		F_goal(3,3)+=gripper_offset;

		geometry_msgs::Pose goalPose = frame2pose(F_goal);

		//publish to distance above position of puzzle piece
		pub_trajectory.publish(goalPose);

		//add delay to allow ur5 to move. for now. can think of way to check if ur5 is moving. 
		ros::Duration(10).sleep();

		//publish to position of puzzle piece going vertically down.
		F_goal(3,3)-=table_offset;
		goalPose=frame2pose(F_goal);
		pub_trajectory.publish(goalPose);
		ros::Duration(10).sleep();

		//activate vacuum gripper
		pub_gripper.publish(angle_grip);
		ros::Duration(2).sleep();


		//move to intermediate position
		pub_trajectory.publish(home);

		//move to final position of puzzle piece
		//deactivate puzzle piece

	}

public:
	main_controller(ros::NodeHandle& nh): nh(nh){
		gripper_offset=84.3/1000.0;
		table_offset=0.1;
		angle_grip.data=150;
		angle_ungrip.data=0;
		z_distance=0.3;
		stepsize=0.1;
		msg.data=true;
		pub_trajectory=nh.advertise<geometry_msgs::Pose>("/setpoint", 1);
		pub_moved=nh.advertise<std_msgs::Bool>("/moved",1);
		// sub_cameraCal=nh.subscribe("/camerapose", 1, &main_controller::setCameraPose, this);
		setCameraPose();
		travelAcrossPlane();
		// sub_puzzlePiece=nh.subscribe("/feature_matcher/piece_pose", 1, &main_controller::puzzleSolver, this);
		
		// pub_gripper=nh.advertise<std_msgs::UInt16>("/servo",1);

	}

	Eigen::Matrix<double,4,4> pose2frame(geometry_msgs::Pose p)
	{
		tf::Quaternion quat(p.orientation.x, p.orientation.y,p.orientation.z, p.orientation.w);   
	    Eigen::Vector3d pos(p.position.x, p.position.y, p.position.z);
	    Eigen::Matrix<double,3,3> R;
	    R=quat2rotm(quat);
	    Eigen::Matrix<double,4, 4> goalFrame=Eigen::Matrix4d::Identity();
	    goalFrame.block(0,0,3,3)<<R;
    	goalFrame.block(0,3,3,1)<<pos;
    	return goalFrame;
	}

	Eigen::Matrix<double,3,3> quat2rotm(tf::Quaternion q)
  	{
	    float qw=q.w(), qx=q.x(), qy=q.y(), qz=q.z();
	    std::cout<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
	  	Eigen::Matrix<double,3,3> R;
		R(0,0)=1-2*qy*qy-2*qz*qz;
		R(0,1)=2*qx*qy-2*qz*qw;
		R(0,2)=2*qx*qz+2*qy*qw;
	  	R(1,0)=2*qx*qy+2*qz*qw;
	  	R(1,1)=1-2*qx*qx-2*qz*qz;
	  	R(1,2)=2*qy*qz-2*qx*qw;
	  	R(2,0)=2*qx*qz-2*qy*qw;
	  	R(2,1)=2*qy*qz+2*qx*qw;
	  	R(2,2)=1-2*qx*qx-2*qy*qy;
	  	return R;
  }

  geometry_msgs::Pose frame2pose(Eigen::Matrix<double,4,4> f)
  {
  	double qw,qx,qy,qz;
  	qw=sqrt(1+f(0,0)+f(1,1)+f(2,2))/2.0;
  	qx=(f(2,1)-f(1,2))/(4*qw);
  	qy=(f(0,2)-f(2,0))/(4*qw);
  	qx=(f(1,0)-f(0,1))/(4*qw);
  	geometry_msgs::Pose g;
  	g.orientation.x=qx;
  	g.orientation.y=qy;
  	g.orientation.z=qz;
  	g.orientation.w=qw;
  	g.position.x=f(0,3);
  	g.position.y=f(1,3);
  	g.position.z=f(2,3);

  	// g.orientation={qw,qx,qy,qz};
  	// g.position={f(0,3),f(1,3),f(2,3)};
  	return g;
  }

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "main_controller");
	ros::NodeHandle nh;
	main_controller maincontrol (nh) ;
	ros::spin();
	return 0;
}