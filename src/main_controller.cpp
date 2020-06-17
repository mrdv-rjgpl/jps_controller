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
      this->placePiece();
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
    if((this->piece_gripped == false) && (this->piece_centered == false))
    {
      image_pose = p;
      this->piece_in_frame = true;
      sscanf(p.header.frame_id.c_str(), "%d", &piece_index);
      ROS_INFO_STREAM("Piece " << this->piece_index << " found in frame during image callback.\n");
    }
    else
    {
      // No operation
    }
  }

  void placePiece(void)
  {
    tf::StampedTransform base_ee_transform;
    jps_traveler::MotionWithTime msg;

    if(this->piece_index >= 0)
    {
      try
      {
        // Move the EE to a pre-programmed position based on the piece index.
        listener.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/base_link", "/ee_link", ros::Time(0), base_ee_transform);
        msg.pose = this->piece_goal_poses[this->piece_index];
        msg.sec = 6;
        pub_trajectory.publish(msg);
        ros::Duration(msg.sec + 2).sleep();

        // Move the EE down to the pad.
        msg.pose.position.z = this->ground_z;
        msg.sec = 4;
        pub_trajectory.publish(msg);
        ros::Duration(msg.sec + 2).sleep();

        // Deactivate the vacuum gripper
        this->pub_gripper.publish(angle_ungrip);
        ros::Duration(2).sleep();
        this->piece_gripped = false;

        // Raise the EE back up to the original position.
        msg.pose = this->piece_goal_poses[this->piece_index];
        msg.sec = 6;
        pub_trajectory.publish(msg);
        ros::Duration(msg.sec + 2).sleep();
        this->num_pieces_placed++;
        this->piece_index = -1;
        this->piece_centered = false;
        this->piece_in_frame = false;

        msg.pose = this->home;
        msg.sec = 6;
        pub_trajectory.publish(msg);
        ros::Duration(msg.sec + 2).sleep();
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR_STREAM(ex.what());
      }
    }
    else
    {
      // Deactivate the vacuum gripper
      this->pub_gripper.publish(angle_ungrip);
      ros::Duration(2).sleep();
      this->piece_gripped = false;
      ROS_ERROR_STREAM("Invalid piece index: " << this->piece_index);
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

      Eigen::Matrix<double,4,4> base_ee_mat = transformToMatrix(base_ee_transform);
      Eigen::Matrix<double,4,1> puzzle_offset_ee(0.0, -0.005, -0.005, 0.0);
      Eigen::Matrix<double, 4, 1> puzzle_offset_base = base_ee_mat * puzzle_offset_ee;

      goal_pose.position.x = base_cam_transform.getOrigin().x() + puzzle_offset_base(0);
      goal_pose.position.y = base_cam_transform.getOrigin().y() + puzzle_offset_base(1);
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
    if ((image_pose.pose.position.z >= 5) || (theta >= 0.05) || (theta <= -0.05))
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
    this->piece_in_frame = false;
  }

  void travel(void)
  {
    double ee_distance;
    jps_traveler::MotionWithTime msg;
    tf::StampedTransform base_ee_transform;

    if(this->robot_pose_index < 48)
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

        if(this->robot_pose_index % 8 == 0)
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
    this->home.position.x = -0.05;
    this->home.position.y = 0.430;
    this->home.position.z = 0.315;
    this->home.orientation.x = -0.616;
    this->home.orientation.y = 0.344;
    this->home.orientation.z = 0.619;
    this->home.orientation.w = 0.346;

    this->travel_pose = this->home;
    this->ground_z = 0.195;

    for(i = 0; i < 4; ++i)
    {
      this->piece_goal_poses[i].position.z = this->home.position.z;
      this->piece_goal_poses[i].orientation.x = -0.013;
      this->piece_goal_poses[i].orientation.y = 0.710;
      this->piece_goal_poses[i].orientation.z = 0.014;
      this->piece_goal_poses[i].orientation.w = 0.704;
    }

    this->piece_goal_poses[0].position.x = 0.283;
    this->piece_goal_poses[0].position.y = 0.438;
    this->piece_goal_poses[1].position.x = 0.331;
    this->piece_goal_poses[1].position.y = 0.484;
    this->piece_goal_poses[2].position.x = 0.329;
    this->piece_goal_poses[2].position.y = 0.387;
    this->piece_goal_poses[3].position.x = 0.376;
    this->piece_goal_poses[3].position.y = 0.436;

    gripper_offset = 84.3 / 1000.0;
    table_offset = 0.1;
    angle_grip.data = 150;
    angle_ungrip.data = 0;
    z_distance = 0.3;
    stepsize = 0.1;
    msg.data = true;

    ROS_INFO("Initializing publisher for robot...");
    pub_trajectory = nh.advertise<jps_traveler::MotionWithTime>("/setpoint", 1);
    ROS_INFO("Initializing publisher for gripper...");
    this->pub_gripper = this->nh.advertise<std_msgs::UInt16>("/servo", 1);
    ros::Duration(5).sleep();
    ROS_INFO("Opening gripper...");
    this->pub_gripper.publish(angle_ungrip);
    ros::Duration(2).sleep();
    ROS_INFO("Homing robot...");
    this->sendToHome();

    ROS_INFO("Initializing SURF feature subscriber...");
    sub_puzzlePiece = this->nh.subscribe(
        "/feature_matcher/homographic_transform",
        1,
        &MainController::getSurfData,
        this);
    ROS_INFO("Initializing timer...");
    timer = this->nh.createTimer(ros::Duration(6.0), &MainController::runRobot, this);
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

