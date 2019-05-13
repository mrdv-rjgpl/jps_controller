#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <jps_traveler/MotionWithTime.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ur_kinematics/ur_kin.h>

class MainController
{
private:
  ros::NodeHandle nh;
  // ros::Subscriber sub_cameraCal;
  ros::Subscriber sub_puzzlePiece;
  ros::Publisher pub_trajectory;
  ros::Publisher pub_gripper;

  ros::Timer timer;
  int num_pieces_placed;
  bool piece_gripped;
  bool piece_centered;
  bool piece_in_frame;
  double ground_z;
  tf::TransformListener listener;

  // constants needed
  geometry_msgs::PoseStamped image_pose;
  geometry_msgs::Pose home;
  geometry_msgs::Pose travel_pose;
  geometry_msgs::Pose piece_goal_poses[4];
  double robot_delta_x;
  double robot_delta_y;
  int robot_pose_index;
  int piece_index;
  double gripper_offset;
  double table_offset;
  std_msgs::UInt16 angle_grip;
  std_msgs::UInt16 angle_ungrip;
  double z_distance;
  double stepsize;
  std_msgs::Bool msg;

  void sendToHome()
  {
    // pose.position=(0.266, 0.422, 0.432);
    // pose.orientation(-0.271, 0.653, 0.272, 0.653);
    // tf::Quaternion quat(-0.271, 0.653, 0.272, 0.653);
    // Eigen::Vector3d pos(0.266, 0.422, 0.432);
    jps_traveler::MotionWithTime m;
    m.pose = this->home;
    m.sec = 6;
    pub_trajectory.publish(m);
    ros::Duration(m.sec + 2).sleep();
  }

  void runRobot(const ros::TimerEvent&)
  {
    if (this->num_pieces_placed >= 4)
    {
      ROS_INFO("All pieces successfully placed.\n");
      // No operation
    }
    else if (this->piece_gripped)
    {
      ROS_INFO("Placing piece...\n");
      //this->placePiece();
    }
    else if (this->piece_centered)
    {
      ROS_INFO("Picking piece...\n");
      this->pickPiece();
    }
    else if (this->piece_in_frame)
    {
      ROS_INFO("Centering camera on piece...\n");
      this->centerPiece();
    }
    else
    {
      ROS_INFO("Traveling across workspace...\n");
      this->travel();
    }

    ROS_INFO("Robot run complete.");
  }

  void getSurfData(const geometry_msgs::PoseStamped p)
  {
    image_pose = p;
    this->piece_in_frame = true;
    sscanf(p.header.frame_id.c_str(), "%d", &piece_index);
    ROS_INFO("Piece found in frame during image callback.\n");
  }

  void placePiece(void)
  {
    tf::StampedTransform base_ee_transform;
    jps_traveler::MotionWithTime msg;
    geometry_msgs::Pose goal_pose;

    if(this->piece_index >= 0)
    {
      // TODO: Move the EE to a pre-programmed position based on the piece index.
      try
      {
        listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);
        goal_pose = this->piece_goal_poses[this->piece_index];
        msg.pose = goal_pose;
        msg.sec = 6;
        pub_trajectory.publish(msg);
        ros::Duration(m.sec + 2).sleep();
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR_STREAM(ex.what());
      }
    }
    else
    {
      ROS_ERROR_STREAM("Invalid piece index: " << this->piece_index);
    }

    // Deactivate the vacuum gripper
    this->pub_gripper.publish(angle_ungrip);
    ros::Duration(2).sleep();
    this->piece_gripped = false;

    if(this->piece_index >= 0)
    {
      this->num_pieces_placed++;
      this->piece_index = -1;
    }
    else
    {
      // No operation
    }
  }

  void pickPiece(void)
  {
    tf::StampedTransform base_cam_transform;
    tf::StampedTransform base_ee_transform;
    geometry_msgs::Pose goal_pose;
    jps_traveler::MotionWithTime msg;
    this->piece_centered = false;
    this->piece_in_frame = false;

    ROS_INFO_STREAM(
        "Camera centered on puzzle piece with error of "
        << image_pose.pose.position.z
        << " pixels.");

    try
    {
      listener.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(10.0));
      listener.lookupTransform("/base_link", "/camera_link", ros::Time(0), base_cam_transform);

      listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
      listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);

      goal_pose.position.x = base_cam_transform.getOrigin().x() + 0.006;
      goal_pose.position.y = base_cam_transform.getOrigin().y() - 0.002;
      goal_pose.position.z = base_cam_transform.getOrigin().z();
      goal_pose.orientation.x = base_ee_transform.getRotation().x();
      goal_pose.orientation.y = base_ee_transform.getRotation().y();
      goal_pose.orientation.z = base_ee_transform.getRotation().z();
      goal_pose.orientation.w = base_ee_transform.getRotation().w();

      ROS_INFO_STREAM("Translating EE to camera position\n" << goal_pose);
      msg.pose = goal_pose;
      msg.sec = 3;
      pub_trajectory.publish(msg);
      ros::Duration(msg.sec + 2).sleep();

      goal_pose.position.z = this->ground_z;
      msg.pose = goal_pose;
      ROS_INFO_STREAM("Translating EE to puzzle centroid\n" << goal_pose);
      pub_trajectory.publish(msg);
      msg.sec = 5;
      ros::Duration(msg.sec + 2).sleep();

      ROS_INFO_STREAM("Gripping puzzle piece...");
      pub_gripper.publish(angle_grip);
      ros::Duration(2).sleep();
      this->sendToHome();
      this->piece_gripped = true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR_STREAM(ex.what());
    }
  }

  void centerPiece(void)
  {
    double theta = -2.0 * atan2(image_pose.pose.orientation.z, image_pose.pose.orientation.w);
    double theta_gain = 0.9;
    double x_gain = 1.0 / 10000.0;
    double y_gain = 1.0 / 10000.0;
    Eigen::Quaterniond goal_q;
    geometry_msgs::Pose goal_pose;
    tf::StampedTransform ee_base_tf;
    tf::StampedTransform cam_ee_tf;
    jps_traveler::MotionWithTime msg;

    ROS_INFO_STREAM("Error norm: " << image_pose.pose.position.z << ", Error angle: " << theta);

    // Check the centering error in pixels and radians
    if ((image_pose.pose.position.z >= 2) || (theta >= 0.05) || (theta <= -0.05))
    {
      ROS_INFO_STREAM("Centering camera over puzzle piece...");

      try
      {
        // Obtain the transformation from the EE link to the base.
        listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), ee_base_tf);

        // Obtain the transformation from the camera link to the EE link.
        listener.waitForTransform("/ee_link", "/camera_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/ee_link", "/camera_link", ros::Time(0), cam_ee_tf);

        // Compute the required translation of the camera link.
        Eigen::Matrix<double, 4, 1> trans_cam_vec(
            x_gain * image_pose.pose.position.x,
            y_gain * image_pose.pose.position.y,
            0,
            1);
        ROS_INFO_STREAM("Required translation in camera frame:\n" << trans_cam_vec);

        // Use the same vector rotated to the EE frame to compute the
        // required translation for the EE link.
        Eigen::Matrix<double, 4, 4> cam_ee_mat = transformToMatrix(cam_ee_tf);
        cam_ee_mat(0, 3) = 0;
        cam_ee_mat(1, 3) = 0;
        cam_ee_mat(2, 3) = 0;
        Eigen::Matrix<double, 4, 1> trans_ee_vec = cam_ee_mat * trans_cam_vec;
        ROS_INFO_STREAM("Required translation in EE frame:\n" << trans_ee_vec);

        // Rotate the vector to the base link frame to compute the
        // required translation of the EE link in the base frame.
        // At this point, the current position of the base link is yet to
        // be accounted for.
        Eigen::Matrix<double, 4, 4> ee_base_mat = transformToMatrix(ee_base_tf);
        ee_base_mat(0, 3) = 0;
        ee_base_mat(1, 3) = 0;
        ee_base_mat(2, 3) = 0;
        Eigen::Matrix<double, 4, 1> trans_base_vec = ee_base_mat * trans_ee_vec;

        // Saturate to 10 cm for safety reasons
        double trans_base_vec_norm = sqrt(
            (trans_base_vec(0, 0) * trans_base_vec(0, 0))
            + (trans_base_vec(1, 0) * trans_base_vec(1, 0)));

        if(trans_base_vec_norm > 0.1)
        {
          trans_base_vec(0, 0) *= 0.1 / trans_base_vec_norm;
          trans_base_vec(1, 0) *= 0.1 / trans_base_vec_norm;
        }
        else
        {
          // No operation
        }

        ROS_INFO_STREAM("Required translation in base frame:\n" << trans_base_vec);

        // Compute the required rotation in the camera frame.
        theta *= theta_gain;

        // Saturate to 10 degrees, if required.
        if (theta >= 0.174)
        {
          theta = 0.174;
        }
        else if (theta <= -0.174)
        {
          theta = -0.174;
        }

        // Obtain rotation from computed angle.
        Eigen::Quaterniond q_x(cos(theta / 2), 0, 0, sin(theta / 2));
        Eigen::Quaterniond q_curr(
            ee_base_tf.getRotation().w(),
            ee_base_tf.getRotation().x(),
            ee_base_tf.getRotation().y(),
            ee_base_tf.getRotation().z());

        goal_q.setIdentity();
        goal_q.w() = q_x.w() * q_curr.w() - q_x.vec().dot(q_curr.vec());
        goal_q.vec() = q_x.w() * q_curr.vec() + q_curr.w() * q_x.vec() + q_x.vec().cross(q_curr.vec());

        if (goal_q.w() < 0)
        {
          goal_q.w() *= -1;
          goal_q.x() *= -1;
          goal_q.y() *= -1;
          goal_q.z() *= -1;
        }
        else
        {
          // No operation
        }

        goal_q.normalize();

        // We only want to move in the x and y directions. Keep the z constant.
        goal_pose.position.x = trans_base_vec(0, 0) + ee_base_tf.getOrigin().x();
        goal_pose.position.y = trans_base_vec(1, 0) + ee_base_tf.getOrigin().y();
        goal_pose.position.z = ee_base_tf.getOrigin().z();
        goal_pose.orientation.x = goal_q.x();
        goal_pose.orientation.y = goal_q.y();
        goal_pose.orientation.z = goal_q.z();
        goal_pose.orientation.w = goal_q.w();

        ROS_INFO_STREAM("Goal pose in base frame after translation and rotation:\n" << goal_pose);

        msg.pose = goal_pose;
        msg.sec = 2;
        pub_trajectory.publish(msg);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR_STREAM(ex.what());
      }
    }
    else
    {
      this->piece_centered = true;
    }

    // Reset the piece in frame boolean, as after this movement,
    // it may no longer be true.
  }

  void travel(void)
  {
    double ee_distance;
    jps_traveler::MotionWithTime msg;
    tf::StampedTransform base_ee_transform;

    if(this->robot_pose_index < 16)
    {
      try
      {
        // Move EE to next pose in pre-programmed sequence.
        this->listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
        this->listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);

        // Compute the distance to be traveled to determine the required speed.
        msg.pose = this->travel_pose;
        ee_distance = sqrt(
          (this->travel_pose.position.x - base_ee_transform.getOrigin().x()) * (this->travel_pose.position.x - base_ee_transform.getOrigin().x())
          + (this->travel_pose.position.y - base_ee_transform.getOrigin().y()) * (this->travel_pose.position.y - base_ee_transform.getOrigin().y())
          + (this->travel_pose.position.z - base_ee_transform.getOrigin().z()) * (this->travel_pose.position.z - base_ee_transform.getOrigin().z()));

        // Limit the speed to 5 cm/s if the distance to be traveled is more than 10 cm.
        if(ee_distance < 0.1)
        {
          msg.sec = 2;
        }
        else
        {
          msg.sec = ee_distance / 0.05;
        }

        pub_trajectory.publish(msg);

        if(msg.sec > 2)
        {
          ros::Duration(msg.sec + 2).sleep();
        }
        else
        {
          // No operation
        }

        // Increment pose counter and update next travel pose.
        this->robot_pose_index++;

        if(this->robot_pose_index % 4 == 0)
        {
          this->travel_pose.position.x += this->robot_delta_x;
          this->robot_delta_y *= -1.0;
        }
        else
        {
          this->travel_pose.position.y += this->robot_delta_y;
        }
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR_STREAM(ex.what());
      }
    }
    else
    {
      ROS_INFO("The entire workspace has been traveled across.");
    }
  }

public:
  MainController(ros::NodeHandle& nh) : nh(nh)
  {
    int i;

    this->num_pieces_placed = 0;
    this->piece_gripped = false;
    this->piece_centered = false;
    this->piece_in_frame = false;
    this->piece_index = -1;
    this->robot_pose_index = 0;
    this->robot_delta_x = 3.0 / 100.0;
    this->robot_delta_y = 3.0 / 100.0;

    // Initialize robot home and initial travel pose.
    this->home.position.x = -0.0;
    this->home.position.y = 0.430;
    this->home.position.z = 0.315;
    this->home.orientation.x = -0.616;
    this->home.orientation.y = 0.344;
    this->home.orientation.z = 0.619;
    this->home.orientation.w = 0.346;

    this->travel_pose = this->home;
    this->ground_z = 0.197;

    for(i = 0; i < 4; ++i)
    {
      this->piece_goal_poses[i].position.z = this->home.position.z;
      this->piece_goal_poses[i].orientation.x = -0.013;
      this->piece_goal_poses[i].orientation.y = 0.710;
      this->piece_goal_poses[i].orientation.z = 0.014;
      this->piece_goal_poses[i].orientation.w = 0.704;
    }

    this->piece_goal_poses[0].position.x = 0.376;
    this->piece_goal_poses[0].position.y = 0.436;
    this->piece_goal_poses[1].position.x = 0.376;
    this->piece_goal_poses[1].position.y = 0.436;
    this->piece_goal_poses[2].position.x = 0.376;
    this->piece_goal_poses[2].position.y = 0.436;
    this->piece_goal_poses[3].position.x = 0.376;
    this->piece_goal_poses[3].position.y = 0.436;

    gripper_offset = 84.3 / 1000.0;
    table_offset = 0.1;
    angle_grip.data = 150;
    angle_ungrip.data = 0;
    z_distance = 0.3;
    stepsize = 0.1;
    msg.data = true;
    pub_trajectory = nh.advertise<jps_traveler::MotionWithTime>("/setpoint", 1);
    this->sendToHome();

    ROS_INFO("Initializing publisher for gripper...");
    this->pub_gripper = this->nh.advertise<std_msgs::UInt16>("/servo", 1);
    ros::Duration(5).sleep();
    ROS_INFO("Opening gripper...");
    this->pub_gripper.publish(angle_ungrip);
    ros::Duration(2).sleep();
    ROS_INFO("Initializing SURF feature subscriber...");
    sub_puzzlePiece = this->nh.subscribe(
        "/feature_matcher/homographic_transform",
        1,
        &MainController::getSurfData,
        this);
    ROS_INFO("Initializing timer...");
    timer = this->nh.createTimer(ros::Duration(4.0), &MainController::runRobot, this);
  }

  Eigen::Matrix<double, 4, 4> transformToMatrix(tf::StampedTransform t)
  {
    tf::Quaternion q(t.getRotation().x(), t.getRotation().y(), t.getRotation().z(), t.getRotation().w());
    Eigen::Vector3d p_vec(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z());
    Eigen::Matrix<double, 3, 3> rot_mat = quat2rotm(q);
    Eigen::Matrix<double, 4, 4> transform_mat = Eigen::Matrix4d::Identity();
    transform_mat.block(0, 0, 3, 3) << rot_mat;
    transform_mat.block(0, 3, 3, 1) << p_vec;

    return transform_mat;
  }

  Eigen::Matrix<double, 4, 4> poseToMatrix(geometry_msgs::Pose p)
  {
    tf::Quaternion quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    Eigen::Vector3d pos(p.position.x, p.position.y, p.position.z);
    Eigen::Matrix<double, 3, 3> R;
    R = quat2rotm(quat);
    Eigen::Matrix<double, 4, 4> goalFrame = Eigen::Matrix4d::Identity();
    goalFrame.block(0, 0, 3, 3) << R;
    goalFrame.block(0, 3, 3, 1) << pos;
    return goalFrame;
  }

  Eigen::Matrix<double, 3, 3> quat2rotm(tf::Quaternion q)
  {
    float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    // std::cout<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
    Eigen::Matrix<double, 3, 3> R;
    R(0, 0) = 1 - 2 * qy * qy - 2 * qz * qz;
    R(0, 1) = 2 * qx * qy - 2 * qz * qw;
    R(0, 2) = 2 * qx * qz + 2 * qy * qw;
    R(1, 0) = 2 * qx * qy + 2 * qz * qw;
    R(1, 1) = 1 - 2 * qx * qx - 2 * qz * qz;
    R(1, 2) = 2 * qy * qz - 2 * qx * qw;
    R(2, 0) = 2 * qx * qz - 2 * qy * qw;
    R(2, 1) = 2 * qy * qz + 2 * qx * qw;
    R(2, 2) = 1 - 2 * qx * qx - 2 * qy * qy;
    return R;
  }

  geometry_msgs::Pose frame2pose(Eigen::Matrix<double, 4, 4> f)
  {
    double qw, qx, qy, qz;
    qw = sqrt(1 + f(0, 0) + f(1, 1) + f(2, 2)) / 2.0;
    qx = (f(2, 1) - f(1, 2)) / (4 * qw);
    qy = (f(0, 2) - f(2, 0)) / (4 * qw);
    qx = (f(1, 0) - f(0, 1)) / (4 * qw);
    geometry_msgs::Pose g;
    g.orientation.x = qx;
    g.orientation.y = qy;
    g.orientation.z = qz;
    g.orientation.w = qw;
    g.position.x = f(0, 3);
    g.position.y = f(1, 3);
    g.position.z = f(2, 3);

    // g.orientation={qw,qx,qy,qz};
    // g.position={f(0,3),f(1,3),f(2,3)};
    return g;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "main_controller");
  ros::NodeHandle nh;
  MainController maincontrol(nh);
  ros::spin();
  return 0;
}

// #include <ros/ros.h>
// #include <ur_kinematics/ur_kin.h>
// //#include "assignment1.hpp"
// #include <Eigen/Eigen>
// #include <Eigen/Dense>
// // #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_listener.h>
// #include <sensor_msgs/JointState.h>
// #include <geometry_msgs/Point.h>
// // #include<geometry_msgs/Pose.h>
// #include <geometry_msgs/Pose.h>
// // #include<rob_param.h>
// #include <trajectory_msgs/JointTrajectory.h>
// #include <math.h>
// #include<std_msgs/UInt16.h>
// #include<std_msgs/Bool.h>
// #include<jps_traveler/MotionWithTime.h>

// class MainController{
//   private:
//     ros::NodeHandle nh;
//     // ros::Subscriber sub_cameraCal;
//     ros::Subscriber sub_puzzlePiece;
//     ros::Publisher pub_trajectory;
//     ros::Publisher pub_gripper;

//     tf::TransformListener listener;

//     //constants needed
//     geometry_msgs::Pose camframe;
//     geometry_msgs::Pose home;
//     double gripper_offset;
//     double table_offset;
//     std_msgs::UInt16 angle_grip;
//     std_msgs::UInt16 angle_ungrip;
//     double z_distance;
//     double stepsize;
//     std_msgs::Bool msg;

//     void travelAcrossPlane()
//     {
//       ros::Duration(5).sleep();
//       // std::cout<<"in travel"<<std::endl;
//       ROS_INFO("in travel");
//       geometry_msgs::Pose pose;
//       pose.position.x=-0.0;
//       pose.position.y=0.430;
//       pose.position.z=0.315;
//       pose.orientation.x=-0.608;
//       pose.orientation.y=0.333;
//       pose.orientation.z=0.625;
//       pose.orientation.w=0.359;
//       home=pose;
//       double temp=pose.position.y;

//       // pose.position=(0.266, 0.422, 0.432);
//       // pose.orientation(-0.271, 0.653, 0.272, 0.653);
//       // tf::Quaternion quat(-0.271, 0.653, 0.272, 0.653);
//       // Eigen::Vector3d pos(0.266, 0.422, 0.432);
//       jps_traveler::MotionWithTime m;
//       m.pose=pose;
//       m.sec=6;
//       pub_trajectory.publish(m);
//       ros::Duration(m.sec+2).sleep();
//       // ros::Duration(2).sleep();
//       // for(int i=1;i<=5;i++)
//       // {
//       // 	for(int j=1;j<=5;j++)
//       // 	// pose.position.x+=1/50.0;
//       // 	{
//       // 		pose.position.y+=3.0/100.0;
//       // 		// std::cout<<pose<<std::endl;
//       // 		ROS_INFO_STREAM(pose);
//       // 		pub_trajectory.publish(pose);
//       // 		ros::Duration(10).sleep();
//       // 		ros::Duration(2).sleep();
//       // 		// std::cout<<"pose executed"<<std::endl;
//       // 		ROS_INFO_STREAM("pose executed");
//       // 	}
//       // 	pose.position.x+=3.0/100.0;
//       // 	pose.position.y=temp;
//       // 	pub_trajectory.publish(pose);
//       // 	ros::Duration(10).sleep();
//       // 	ros::Duration(2).sleep();
//       // }
//       // std::cout<<"after everything"<<std::endl;
//       ROS_INFO_STREAM("after everything");

//     }

//     //get transform between ee and camera. After hand-eye cal.
//     void setCameraPose()
//     {
//       // camframe=p;
//       // camframe.position.x=p.position.x;
//     }

//     void findImageCenter(const geometry_msgs::PoseStamped image_pose)
//     {
//       double x_gain = 1.0 / 10000.0;
//       double y_gain = 1.0 / 10000.0;
//       double theta_gain = 0.9;

//       double theta = -2.0 * atan2(image_pose.pose.orientation.z, image_pose.pose.orientation.w);
//       ROS_INFO_STREAM("Obtained differential angle of " << theta);

//       if(image_pose.pose.position.z < 2 && theta < 0.05 && theta > -0.05)
//       {
//         msg.data=true;

//         ROS_INFO_STREAM("over center in cam coord  "<<image_pose.pose.position.z);
//         tf::StampedTransform base_cam_transform;

//         try
//         {
//           listener.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(10.0));
//           listener.lookupTransform("/base_link", "/camera_link", ros::Time(0), base_cam_transform);
//         }
//         catch (tf::TransformException ex) {
//           ROS_ERROR("Not found base to cam");
//           ros::Duration(1.0).sleep();
//         }

//         tf::StampedTransform base_ee_transform;

//         try
//         {
//           listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
//           listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);
//         }
//         catch (tf::TransformException ex) {
//           ROS_ERROR("Not found base to ee");
//           ros::Duration(1.0).sleep();
//         }
//         geometry_msgs::Pose goalPose;
//         goalPose.position.x=base_cam_transform.getOrigin().x();
//         goalPose.position.y=base_cam_transform.getOrigin().y();
//         goalPose.position.z=base_cam_transform.getOrigin().z();
//         goalPose.orientation.x=base_ee_transform.getRotation().x();
//         goalPose.orientation.y=base_ee_transform.getRotation().y();
//         goalPose.orientation.z=base_ee_transform.getRotation().z();
//         goalPose.orientation.w=base_ee_transform.getRotation().w();

//         ROS_INFO_STREAM("sending to camera position"<<goalPose);
//         ros::Duration(3).sleep();

//         jps_traveler::MotionWithTime m;
//         m.pose=goalPose;
//         m.sec=8;
//         pub_trajectory.publish(m);
//         ros::Duration(m.sec+2).sleep();

//         goalPose.position.x+=0.003;
//         goalPose.position.y-=0.003;
//         goalPose.position.z=0.199;
//         m.pose=goalPose;
//         ROS_INFO_STREAM("sending to puzzle position"<<goalPose);
//         ros::Duration(3).sleep();
//         pub_trajectory.publish(m);
//         ros::Duration(m.sec+2).sleep();

//   	    pub_gripper.publish(angle_grip);

//       }
//       else
//       {
//         ROS_INFO_STREAM("not over camera");
//         tf::StampedTransform base_ee_transform;
//         try
//         {
//           listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
//           listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);
//         }
//         catch (tf::TransformException ex) {
//           ROS_ERROR("Not found base to ee");
//           ros::Duration(1.0).sleep();
//         }
//         geometry_msgs::Pose goalPose;

//         tf::StampedTransform camera_ee_transform;
//         try
//         {
//           listener.waitForTransform("/ee_link", "/camera_link", ros::Time(0), ros::Duration(10.0));
//           listener.lookupTransform("/ee_link", "/camera_link", ros::Time(0), camera_ee_transform);
//         }
//         catch (tf::TransformException ex) {
//           ROS_ERROR("Not found camera to base");
//           ros::Duration(1.0).sleep();
//         }

//         geometry_msgs::Pose temp;
//         temp.position.x=camera_ee_transform.getOrigin().x();
//         temp.position.y=camera_ee_transform.getOrigin().y();
//         temp.position.z=camera_ee_transform.getOrigin().z();
//         temp.orientation.x=camera_ee_transform.getRotation().x();
//         temp.orientation.y=camera_ee_transform.getRotation().y();
//         temp.orientation.z=camera_ee_transform.getRotation().z();
//         temp.orientation.w=camera_ee_transform.getRotation().w();

//         Eigen::Matrix<double,4,4> cam2ee = poseToMatrix(temp);
//         cam2ee(0,3)=0;cam2ee(1,3)=0;cam2ee(2,3)=0;
//         Eigen::Matrix<double,4,1> trans_camCoord(x_gain*image_pose.pose.position.x,y_gain*image_pose.pose.position.y,
//         0, 1);
//         Eigen::Matrix<double,4,1> trans_eeCoord = cam2ee * trans_camCoord;

//         ROS_INFO_STREAM("translation in cam coord\n" << trans_camCoord);
//         ROS_INFO_STREAM("translation in ee coord\n" << trans_eeCoord);

//         temp.position.x=base_ee_transform.getOrigin().x();
//         temp.position.y=base_ee_transform.getOrigin().y();
//         temp.position.z=base_ee_transform.getOrigin().z();
//         temp.orientation.x=base_ee_transform.getRotation().x();
//         temp.orientation.y=base_ee_transform.getRotation().y();
//         temp.orientation.z=base_ee_transform.getRotation().z();
//         temp.orientation.w=base_ee_transform.getRotation().w();

//         Eigen::Matrix<double,4,4> ee2base = poseToMatrix(temp);
//         ee2base(0,3)=0;ee2base(1,3)=0;ee2base(2,3)=0;
//         Eigen::Matrix<double,4,1> trans_baseCoord = ee2base * trans_eeCoord;

//         ROS_INFO_STREAM("translation in base coord\n" << trans_baseCoord);

//         theta *= theta_gain;

//         if(theta >= 0.174)
//         {
//           theta = 0.174;
//         }
//         else if(theta <= -0.174)
//         {
//           theta = -0.174;
//         }

//         Eigen::Quaterniond q_x(cos(theta/2), 0, 0, sin(theta/2));
//         Eigen::Quaterniond q_curr(base_ee_transform.getRotation().w(), base_ee_transform.getRotation().x(),
//             base_ee_transform.getRotation().y(),
//             base_ee_transform.getRotation().z());

//         Eigen::Quaterniond resultQ;
//         resultQ.setIdentity();

//         resultQ.w() = q_x.w() * q_curr.w() - q_x.vec().dot(q_curr.vec());
//         resultQ.vec() = q_x.w() * q_curr.vec() + q_curr.w() * q_x.vec()
//           + q_x.vec().cross(q_curr.vec());

//         if(resultQ.w() < 0)
//         {
//           resultQ.w() *= -1;
//           resultQ.x()*=-1;
//           resultQ.y()*=-1;
//           resultQ.z()*=-1;
//         }

//         resultQ.normalize();

//         //we only want to move in x and y. everything else remains as is.
//         goalPose.position.x=trans_baseCoord(0,0) + base_ee_transform.getOrigin().x();
//         goalPose.position.y=trans_baseCoord(1,0) + base_ee_transform.getOrigin().y();
//         goalPose.position.z=base_ee_transform.getOrigin().z();
//         // goalPose.orientation.x=base_ee_transform.getRotation().x();
//         // goalPose.orientation.y=base_ee_transform.getRotation().y();
//         // goalPose.orientation.z=base_ee_transform.getRotation().z();
//         // goalPose.orientation.w=base_ee_transform.getRotation().w();
//         // ROS_INFO_STREAM("goal pose in base coord after translation: "<< goalPose);

//         // jps_traveler::MotionWithTime m;
//         // m.pose=goalPose;
//         // m.sec=8;
//         // pub_trajectory.publish(m);
//         // ros::Duration(m.sec+2).sleep();

//         // goalPose.position.x=base_ee_transform.getOrigin().x();
//         // goalPose.position.y=base_ee_transform.getOrigin().y();
//         // goalPose.position.z=base_ee_transform.getOrigin().z();
//         goalPose.orientation.x=resultQ.x();
//         goalPose.orientation.y=resultQ.y();
//         goalPose.orientation.z=resultQ.z();
//         goalPose.orientation.w=resultQ.w();

//         /*
//         ROS_INFO_STREAM("Initial quaternion: ["
//             << q_curr.x() << ", "
//             << q_curr.y() << ", "
//             << q_curr.z() << ", "
//             << q_curr.w() << "]");
//         ROS_INFO_STREAM("Multiplied quaternion: ["
//             << q_x.x() << ", "
//             << q_x.y() << ", "
//             << q_x.z() << ", "
//             << q_x.w() << "]");
//         ROS_INFO_STREAM("Final quaternion: ["
//             << resultQ.x() << ", "
//             << resultQ.y() << ", "
//             << resultQ.z() << ", "
//             << resultQ.w() << "]"); */

//         ROS_INFO_STREAM("goal pose in base coord after rotation: "<<goalPose);
//         m.pose=goalPose;

//         pub_trajectory.publish(m);
//         ros::Duration(10).sleep();

//         ROS_INFO_STREAM("curr pose in base coord\n"
//             << base_ee_transform.getOrigin().x() << "\n"
//             << base_ee_transform.getOrigin().y() << "\n"
//             << base_ee_transform.getOrigin().z() << "\n"
//             << base_ee_transform.getRotation().x() << "\n"
//             << base_ee_transform.getRotation().y() << "\n"
//             << base_ee_transform.getRotation().z() << "\n"
//             << base_ee_transform.getRotation().w() << "\n"
//             );
//         ROS_INFO_STREAM("goal pose in base coord:\n"<< goalPose);
//         // pub_trajectory.publish(goalPose);
//         // ros::Duration(10).sleep();

//         msg.data=true;
//       }
//       // goalPose.x=x_gain*

//     }

//     void getBaseToEE(tf::StampedTransform& base_ee_transform)
//     {
//       try
//       {
//         listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
//         listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);
//       }
//       catch (tf::TransformException ex) {
//         ROS_ERROR("Not found base to ee");
//         ros::Duration(1.0).sleep();
//       }
//     }

//     //pose from base to puzzle
//     void puzzleSolver(const geometry_msgs::PoseStamped& puzzlePose)
//     {
//       //frames base2puzzle, base2ee, ee2camera, camera2puzzle. base -> base_link
//       Eigen::Matrix<double,4,4> F_bp, F_be, F_ec, F_cp;
//       //gripper offset

//       F_bp=poseToMatrix(puzzlePose.pose);
//       // F_ec=poseToMatrix(camframe);
//       // tf::StampedTransform base_ee_transform;
//       // try
//       // {
//       // 	listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
//       // 	listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);
//       // }
//       // catch (tf::TransformException ex) {
//       // 	ROS_ERROR("Not found");
//       // 	ros::Duration(1.0).sleep();

//       // }
//       // //convert from tf::transform to frame. calculate F_bp
//       // // geometry_msgs::Pose base2ee = [base_ee_transform.getOrigin(), base_ee_transform.getRotation()];
//       // geometry_msgs::Pose base2ee;
//       // base2ee.orientation.w = base_ee_transform.getRotation().w();
//       // base2ee.orientation.x = base_ee_transform.getRotation().x();
//       // base2ee.orientation.y = base_ee_transform.getRotation().y();
//       // base2ee.orientation.z = base_ee_transform.getRotation().z();

//       // base2ee.position.x = base_ee_transform.getOrigin().x();
//       // base2ee.position.y = base_ee_transform.getOrigin().y();
//       // base2ee.position.z = base_ee_transform.getOrigin().z();

//       // F_be=poseToMatrix(base2ee);

//       //computes pose from base to puzzle
//       // F_bp = F_be * F_ec * F_cp;

//       //frame from base to vaccuum gripper
//       Eigen::Matrix<double,4,4> F_bvg=F_bp;
//       F_bvg(3,3)+=table_offset;

//       //this is the goal frame from base to ee.
//       Eigen::Matrix<double,4,4> F_goal=F_bvg;
//       F_goal(3,3)+=gripper_offset;

//       geometry_msgs::Pose goalPose = frame2pose(F_goal);

//       //publish to distance above position of puzzle piece
//       pub_trajectory.publish(goalPose);

//       //add delay to allow ur5 to move. for now. can think of way to check if ur5 is moving.
//       ros::Duration(10).sleep();

//       //publish to position of puzzle piece going vertically down.
//       F_goal(3,3)-=table_offset;
//       goalPose=frame2pose(F_goal);
//       pub_trajectory.publish(goalPose);
//       ros::Duration(10).sleep();

//       //activate vacuum gripper
//       pub_gripper.publish(angle_grip);
//       ros::Duration(2).sleep();

//       //move to intermediate position
//       pub_trajectory.publish(home);

//       //move to final position of puzzle piece
//       //deactivate puzzle piece

//     }

//   public:
//     MainController(ros::NodeHandle& nh): nh(nh)
//   {
//       gripper_offset=84.3/1000.0;
//       table_offset=0.1;
//       angle_grip.data=150;
//       angle_ungrip.data=0;
//       z_distance=0.3;
//       stepsize=0.1;
//       msg.data=true;
//       pub_trajectory=nh.advertise<jps_traveler::MotionWithTime>("/setpoint", 1);
//       // sub_cameraCal=nh.subscribe("/camerapose", 1, &MainController::setCameraPose, this);
//       setCameraPose();
//       travelAcrossPlane();
//       // sub_puzzlePiece=nh.subscribe("/feature_matcher/piece_pose", 1, &MainController::puzzleSolver, this);
//       sub_puzzlePiece=nh.subscribe("/feature_matcher/homographic_transform", 1, &MainController::findImageCenter,
//       this);

//       // pub_gripper=nh.advertise<std_msgs::UInt16>("/servo",1);
//       pub_gripper=nh.advertise<jps_traveler::MotionWithTime>("/servo",1);

//     }

//     Eigen::Matrix<double,4,4> poseToMatrix(geometry_msgs::Pose p)
//     {
//       tf::Quaternion quat(p.orientation.x, p.orientation.y,p.orientation.z, p.orientation.w);
//       Eigen::Vector3d pos(p.position.x, p.position.y, p.position.z);
//       Eigen::Matrix<double,3,3> R;
//       R=quat2rotm(quat);
//       Eigen::Matrix<double,4, 4> goalFrame=Eigen::Matrix4d::Identity();
//       goalFrame.block(0,0,3,3)<<R;
//       goalFrame.block(0,3,3,1)<<pos;
//       return goalFrame;
//     }

//     Eigen::Matrix<double,3,3> quat2rotm(tf::Quaternion q)
//     {
//       float qw=q.w(), qx=q.x(), qy=q.y(), qz=q.z();
//       // std::cout<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
//       Eigen::Matrix<double,3,3> R;
//       R(0,0)=1-2*qy*qy-2*qz*qz;
//       R(0,1)=2*qx*qy-2*qz*qw;
//       R(0,2)=2*qx*qz+2*qy*qw;
//       R(1,0)=2*qx*qy+2*qz*qw;
//       R(1,1)=1-2*qx*qx-2*qz*qz;
//       R(1,2)=2*qy*qz-2*qx*qw;
//       R(2,0)=2*qx*qz-2*qy*qw;
//       R(2,1)=2*qy*qz+2*qx*qw;
//       R(2,2)=1-2*qx*qx-2*qy*qy;
//       return R;
//     }

//     geometry_msgs::Pose frame2pose(Eigen::Matrix<double,4,4> f)
//     {
//       double qw,qx,qy,qz;
//       qw=sqrt(1+f(0,0)+f(1,1)+f(2,2))/2.0;
//       qx=(f(2,1)-f(1,2))/(4*qw);
//       qy=(f(0,2)-f(2,0))/(4*qw);
//       qx=(f(1,0)-f(0,1))/(4*qw);
//       geometry_msgs::Pose g;
//       g.orientation.x=qx;
//       g.orientation.y=qy;
//       g.orientation.z=qz;
//       g.orientation.w=qw;
//       g.position.x=f(0,3);
//       g.position.y=f(1,3);
//       g.position.z=f(2,3);

//       // g.orientation={qw,qx,qy,qz};
//       // g.position={f(0,3),f(1,3),f(2,3)};
//       return g;
//     }

// };

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "main_controller");
//   ros::NodeHandle nh;
//   MainController maincontrol (nh) ;
//   ros::spin();
//   return 0;
// }
