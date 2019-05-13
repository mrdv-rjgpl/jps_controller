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
#include<jps_traveler/MotionWithTime.h>


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
	    // std::cout<<"in travel"<<std::endl;
	    ROS_INFO("in travel");
		geometry_msgs::Pose pose;
		pose.position.x=-0.0;
		pose.position.y=0.430;
		pose.position.z=0.315;
		pose.orientation.x=-0.608;
		pose.orientation.y=0.333;
		pose.orientation.z=0.625;
		pose.orientation.w=0.359;
		home=pose;
		double temp=pose.position.y;

		// pose.position=(0.266, 0.422, 0.432);
		// pose.orientation(-0.271, 0.653, 0.272, 0.653);
		// tf::Quaternion quat(-0.271, 0.653, 0.272, 0.653);
		// Eigen::Vector3d pos(0.266, 0.422, 0.432);
		jps_traveler::MotionWithTime m;
		m.pose=pose;
		m.sec=6;
		pub_trajectory.publish(m);
		ros::Duration(m.sec+2).sleep();
		pub_moved.publish(msg);
		// ros::Duration(2).sleep();
		// for(int i=1;i<=5;i++)
		// {
		// 	for(int j=1;j<=5;j++)
		// 	// pose.position.x+=1/50.0;
		// 	{
		// 		pose.position.y+=3.0/100.0;
		// 		// std::cout<<pose<<std::endl;
		// 		ROS_INFO_STREAM(pose);
		// 		pub_trajectory.publish(pose);
		// 		ros::Duration(10).sleep();
		// 		pub_moved.publish(msg);
		// 		ros::Duration(2).sleep();
		// 		// std::cout<<"pose executed"<<std::endl;
		// 		ROS_INFO_STREAM("pose executed");
		// 	}
		// 	pose.position.x+=3.0/100.0;
		// 	pose.position.y=temp;
		// 	pub_trajectory.publish(pose);
		// 	ros::Duration(10).sleep();	
		// 	pub_moved.publish(msg);
		// 	ros::Duration(2).sleep();		
		// }
		// std::cout<<"after everything"<<std::endl;
		ROS_INFO_STREAM("after everything");

	}


	//get transform between ee and camera. After hand-eye cal.
	void setCameraPose()
	{
		// camframe=p;
		// camframe.position.x=p.position.x;
	}





	void findImageCenter(const geometry_msgs::PoseStamped imagePose)
	{
		double x_gain = 1.0 / 10000.0;
		double y_gain = 1.0 / 10000.0;
		double theta_gain = 0.9;

		double theta = -2.0 * atan2(imagePose.pose.orientation.z, imagePose.pose.orientation.w);
    ROS_INFO_STREAM("Obtained differential angle of " << theta);

		if(imagePose.pose.position.z < 2 && theta < 0.05 && theta > -0.05)
		{
			msg.data=true;
			//pub_moved.publish(msg);
			ROS_INFO_STREAM("over center in cam coord  "<<imagePose.pose.position.z);
			tf::StampedTransform base_cam_transform;
			try
			{
				listener.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(10.0));
				listener.lookupTransform("/base_link", "/camera_link", ros::Time(0), base_cam_transform);
			}
			catch (tf::TransformException ex) {
				ROS_ERROR("Not found base to cam");
				ros::Duration(1.0).sleep();
			}

			tf::StampedTransform base_ee_transform;
			try
			{
				listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
				listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);
			}
			catch (tf::TransformException ex) {
				ROS_ERROR("Not found base to ee");
				ros::Duration(1.0).sleep();
			}
			geometry_msgs::Pose goalPose;
			goalPose.position.x=base_cam_transform.getOrigin().x();
			goalPose.position.y=base_cam_transform.getOrigin().y();
			goalPose.position.z=base_cam_transform.getOrigin().z();
			goalPose.orientation.x=base_ee_transform.getRotation().x();
			goalPose.orientation.y=base_ee_transform.getRotation().y();
			goalPose.orientation.z=base_ee_transform.getRotation().z();
			goalPose.orientation.w=base_ee_transform.getRotation().w();

			ROS_INFO_STREAM("sending to camera position"<<goalPose);
			ros::Duration(3).sleep();

			jps_traveler::MotionWithTime m;
			m.pose=goalPose;
			m.sec=8;
			pub_trajectory.publish(m);
			ros::Duration(m.sec+2).sleep();


		}

		else
		{
			ROS_INFO_STREAM("not over camera");
			tf::StampedTransform base_ee_transform;
			try
			{
				listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
				listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);
			}
			catch (tf::TransformException ex) {
				ROS_ERROR("Not found base to ee");
				ros::Duration(1.0).sleep();
			}
			geometry_msgs::Pose goalPose;
			
			tf::StampedTransform camera_ee_transform;
			try
			{
				listener.waitForTransform("/ee_link", "/camera_link", ros::Time(0), ros::Duration(10.0));
				listener.lookupTransform("/ee_link", "/camera_link", ros::Time(0), camera_ee_transform);
			}
			catch (tf::TransformException ex) {
				ROS_ERROR("Not found camera to base");
				ros::Duration(1.0).sleep();
			}

			geometry_msgs::Pose temp;
			temp.position.x=camera_ee_transform.getOrigin().x();
			temp.position.y=camera_ee_transform.getOrigin().y();
			temp.position.z=camera_ee_transform.getOrigin().z();
			temp.orientation.x=camera_ee_transform.getRotation().x();
			temp.orientation.y=camera_ee_transform.getRotation().y();
			temp.orientation.z=camera_ee_transform.getRotation().z();
			temp.orientation.w=camera_ee_transform.getRotation().w();


			Eigen::Matrix<double,4,4> cam2ee = pose2frame(temp);
			cam2ee(0,3)=0;cam2ee(1,3)=0;cam2ee(2,3)=0;
			Eigen::Matrix<double,4,1> trans_camCoord(x_gain*imagePose.pose.position.x,y_gain*imagePose.pose.position.y, 0, 1);
			Eigen::Matrix<double,4,1> trans_eeCoord = cam2ee * trans_camCoord;

			ROS_INFO_STREAM("translation in cam coord\n" << trans_camCoord);
			ROS_INFO_STREAM("translation in ee coord\n" << trans_eeCoord);

			temp.position.x=base_ee_transform.getOrigin().x();
			temp.position.y=base_ee_transform.getOrigin().y();
			temp.position.z=base_ee_transform.getOrigin().z();
			temp.orientation.x=base_ee_transform.getRotation().x();
			temp.orientation.y=base_ee_transform.getRotation().y();
			temp.orientation.z=base_ee_transform.getRotation().z();
			temp.orientation.w=base_ee_transform.getRotation().w();

			Eigen::Matrix<double,4,4> ee2base = pose2frame(temp);
			ee2base(0,3)=0;ee2base(1,3)=0;ee2base(2,3)=0;
			Eigen::Matrix<double,4,1> trans_baseCoord = ee2base * trans_eeCoord;


			ROS_INFO_STREAM("translation in base coord\n" << trans_baseCoord);

			theta *= theta_gain;

      if(theta >= 0.174)
      {
        theta = 0.174;
      }
      else if(theta <= -0.174)
      {
        theta = -0.174;
      }

			Eigen::Quaterniond q_x(cos(theta/2), 0, 0, sin(theta/2));
			Eigen::Quaterniond q_curr(base_ee_transform.getRotation().w(), base_ee_transform.getRotation().x(),
									  base_ee_transform.getRotation().y(),
									  base_ee_transform.getRotation().z());

			Eigen::Quaterniond resultQ;
		    resultQ.setIdentity();

		    resultQ.w() = q_x.w() * q_curr.w() - q_x.vec().dot(q_curr.vec());
		    resultQ.vec() = q_x.w() * q_curr.vec() + q_curr.w() * q_x.vec() 
		    + q_x.vec().cross(q_curr.vec());

			if(resultQ.w() < 0)
			{
				resultQ.w() *= -1;
				resultQ.x()*=-1;
				resultQ.y()*=-1;
				resultQ.z()*=-1;
			}

			resultQ.normalize();



			//we only want to move in x and y. everything else remains as is. 
			goalPose.position.x=trans_baseCoord(0,0) + base_ee_transform.getOrigin().x();
			goalPose.position.y=trans_baseCoord(1,0) + base_ee_transform.getOrigin().y();
			goalPose.position.z=base_ee_transform.getOrigin().z();	
			goalPose.orientation.x=base_ee_transform.getRotation().x();
			goalPose.orientation.y=base_ee_transform.getRotation().y();
			goalPose.orientation.z=base_ee_transform.getRotation().z();
			goalPose.orientation.w=base_ee_transform.getRotation().w();
			ROS_INFO_STREAM("goal pose in base coord after translation: "<< goalPose);

			jps_traveler::MotionWithTime m;
			m.pose=goalPose;
			m.sec=8;
			pub_trajectory.publish(m);
			ros::Duration(m.sec+2).sleep();



			// goalPose.position.x=base_ee_transform.getOrigin().x();
			// goalPose.position.y=base_ee_transform.getOrigin().y();
			// goalPose.position.z=base_ee_transform.getOrigin().z();	
			goalPose.orientation.x=resultQ.x();
			goalPose.orientation.y=resultQ.y();
			goalPose.orientation.z=resultQ.z();
			goalPose.orientation.w=resultQ.w();

      ROS_INFO_STREAM("Initial quaternion: ["
          << q_curr.x() << ", "
          << q_curr.y() << ", "
          << q_curr.z() << ", "
          << q_curr.w() << "]");
      ROS_INFO_STREAM("Multiplied quaternion: ["
          << q_x.x() << ", "
          << q_x.y() << ", "
          << q_x.z() << ", "
          << q_x.w() << "]");
      ROS_INFO_STREAM("Final quaternion: ["
          << resultQ.x() << ", "
          << resultQ.y() << ", "
          << resultQ.z() << ", "
          << resultQ.w() << "]");

			ROS_INFO_STREAM("goal pose in base coord after rotation: "<<goalPose);
			m.pose=goalPose;

			pub_trajectory.publish(m);
			ros::Duration(10).sleep();



			ROS_INFO_STREAM("curr pose in base coord\n"
          << base_ee_transform.getOrigin().x() << "\n"
          << base_ee_transform.getOrigin().y() << "\n"
          << base_ee_transform.getOrigin().z() << "\n"
          << base_ee_transform.getRotation().x() << "\n"
          << base_ee_transform.getRotation().y() << "\n"
          << base_ee_transform.getRotation().z() << "\n"
          << base_ee_transform.getRotation().w() << "\n"
          );
			ROS_INFO_STREAM("goal pose in base coord:\n"<< goalPose);
			// pub_trajectory.publish(goalPose);
			// ros::Duration(10).sleep();

			msg.data=true;
			pub_moved.publish(msg);


		}
		// goalPose.x=x_gain*

	}

	void getBaseToEE(tf::StampedTransform& base_ee_transform)
	{
		try
		{
			listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
			listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);
		}
		catch (tf::TransformException ex) {
			ROS_ERROR("Not found base to ee");
			ros::Duration(1.0).sleep();
		}
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
		pub_trajectory=nh.advertise<jps_traveler::MotionWithTime>("/setpoint", 1);
		pub_moved=nh.advertise<std_msgs::Bool>("/moved",1);
		// sub_cameraCal=nh.subscribe("/camerapose", 1, &main_controller::setCameraPose, this);
		setCameraPose();
		travelAcrossPlane();
		// sub_puzzlePiece=nh.subscribe("/feature_matcher/piece_pose", 1, &main_controller::puzzleSolver, this);
		sub_puzzlePiece=nh.subscribe("/feature_matcher/homographic_transform", 1, &main_controller::findImageCenter, this);

		// pub_gripper=nh.advertise<std_msgs::UInt16>("/servo",1);
		pub_gripper=nh.advertise<jps_traveler::MotionWithTime>("/servo",1);

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
	    // std::cout<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
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
