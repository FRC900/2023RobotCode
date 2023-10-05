#include <talon_state_msgs/TalonState.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <angles/angles.h>

class TalonStateRepublisher
{
    public:
        TalonStateRepublisher(ros::NodeHandle &n):
            sub_(n.subscribe("frcrobot_jetson/talon_states", 1, &TalonStateRepublisher::callback, this)),
            sub_rio_(n.subscribe("/frcrobot_rio/talon_states", 1, &TalonStateRepublisher::rio_callback, this)),
            pub_(n.advertise<sensor_msgs::JointState>("talon_joint_states", 1)) 
            {
                // load swerve drive offsets from /frcrobot_jetson/swerve_drive_controller/steering_joint_bl/offset
                if (!n.getParam("/frcrobot_jetson/swerve_drive_controller/steering_joint_bl/offset", bl_offset)) {
                    ROS_ERROR("Failed to get swerve drive offset for bl");
                }
                if (!n.getParam("/frcrobot_jetson/swerve_drive_controller/steering_joint_br/offset", br_offset)) {
                    ROS_ERROR("Failed to get swerve drive offset for br");
                }
                if (!n.getParam("/frcrobot_jetson/swerve_drive_controller/steering_joint_fl/offset", fl_offset)) {
                    ROS_ERROR("Failed to get swerve drive offset for fl");
                }
                if (!n.getParam("/frcrobot_jetson/swerve_drive_controller/steering_joint_fr/offset", fr_offset)) {
                    ROS_ERROR("Failed to get swerve drive offset for fr");
                }
                // print offsets
                ROS_INFO_STREAM("bl offset: " << bl_offset);
                ROS_INFO_STREAM("br offset: " << br_offset);
                ROS_INFO_STREAM("fl offset: " << fl_offset);
                ROS_INFO_STREAM("fr offset: " << fr_offset);
            }
    

        void rio_callback(const talon_state_msgs::TalonStateConstPtr &msgIn)
        {
            ROS_INFO_STREAM_THROTTLE(1, "Got rio callback");
            sensor_msgs::JointState turret_msgOut;
            turret_msgOut.header.stamp = msgIn->header.stamp;  
            turret_msgOut.header.frame_id = "";
            for (size_t i = 0; i < msgIn->name.size(); i++)
            {
                turret_msgOut.name.push_back(msgIn->name[i]);
                turret_msgOut.position.push_back(msgIn->position[i]);
                turret_msgOut.velocity.push_back(msgIn->speed[i]);
                turret_msgOut.effort.push_back(0);
            }
            pub_.publish(turret_msgOut);
        }


        void callback(const talon_state_msgs::TalonStateConstPtr &msgIn)
        {
            
            sensor_msgs::JointState msgOut;
			msgOut.header.stamp = msgIn->header.stamp;
			msgOut.header.frame_id = "";
            double four_bar_angle;
			
            for (size_t i = 0; i < msgIn->name.size(); i++)
            {
                if(msgIn->name[i]=="bl_angle") {
                    msgOut.position.push_back(angles::normalize_angle_positive(msgIn->position[i] - bl_offset));
                }
                else if(msgIn->name[i]=="br_angle") {
                    msgOut.position.push_back(angles::normalize_angle_positive(msgIn->position[i] - br_offset));
                }
                else if(msgIn->name[i]=="fl_angle") {
                    msgOut.position.push_back(angles::normalize_angle_positive(msgIn->position[i] - fl_offset));
                }
                else if(msgIn->name[i]=="fr_angle") {
                    msgOut.position.push_back(angles::normalize_angle_positive(msgIn->position[i] - fr_offset));
                }
                else if(msgIn->name[i]=="four_bar")
                {
                    four_bar_angle = - msgIn->position[i] + 0.52;
                    msgOut.position.push_back(four_bar_angle);
                }
                else
                {
                    msgOut.position.push_back(msgIn->position[i]);
                }
                
                msgOut.name.push_back(msgIn->name[i]);
                msgOut.velocity.push_back(msgIn->speed[i]);
                msgOut.effort.push_back(0);
            }

            msgOut.name.push_back("joint2");
            msgOut.position.push_back(0 - four_bar_angle);
            msgOut.name.push_back("joint1");
            msgOut.position.push_back(four_bar_angle);

            pub_.publish(msgOut);

        }
    private:
        double bl_offset = 0;
        double br_offset = 0;
        double fl_offset = 0;
        double fr_offset = 0;
        ros::Subscriber sub_;
        ros::Subscriber sub_rio_;
		ros::Publisher  pub_;

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "talon_state_republisher");

	ros::NodeHandle n;
	TalonStateRepublisher talonStateRepublisher(n);

	ros::spin();
	return 0;
}
