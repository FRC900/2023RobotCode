#include <candle_controller_msgs/Colour.h>
#include <candle_controller_msgs/Animation.h>
#include <candle_controller_msgs/Brightness.h>
#include <ros/ros.h>
#include <frc_msgs/ButtonBoxState2023.h>
#include <behavior_actions/AutoMode.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <talon_state_msgs/TalonFXProState.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <frc_msgs/MatchSpecificData.h>
#include <frc_msgs/JoystickState.h>

constexpr uint8_t MAX_LED = 34;
constexpr uint8_t MID_START = 0;
constexpr uint8_t MID_COUNT = 8;
constexpr uint8_t RIGHT_START = 8;
constexpr uint8_t RIGHT_COUNT = 13;
constexpr uint8_t LEFT_START = 21;
constexpr uint8_t LEFT_COUNT = 13;

struct NodeCTX {
    ros::Time last_green_time;
    bool cone_button_pressed;
    bool cube_button_pressed;
    bool updated;
    bool OVERRIDE_GREEN{false};
    bool disabled;
    uint8_t auto_mode;
    double imu_tolerance{M_PI/18.0}; // 10 deg
    bool imu_zeroed;
    size_t intake_idx;
    bool current_exceeded_;
    double intake_current_threshold_;
    bool pathing;
    bool DPADLEFT;
    bool DPADRIGHT;
    bool DPADUP; 
    bool DPADDOWN;

    NodeCTX() :
        cone_button_pressed{false},
        cube_button_pressed{false},
        updated{false},
        disabled{true},
        pathing{false},
        intake_idx{std::numeric_limits<size_t>::max()}
    {}

    void team_colour_callback(const frc_msgs::MatchSpecificData& msg) {
        disabled = msg.Disabled;
        if (msg.Disabled != this->disabled) {
            ROS_INFO_STREAM("Updating auto LEDs...");
            disabled = msg.Disabled;
            updated = true;
        }
    }

    void talon_callback(const talon_state_msgs::TalonFXProState &talon_state)
	{
		// fourbar_master_idx == max of size_t at the start
		if (intake_idx == std::numeric_limits<size_t>::max()) // could maybe just check for > 0
		{
			for (size_t i = 0; i < talon_state.name.size(); i++)
			{
				if (talon_state.name[i] == "intake")
				{
					intake_idx = i;
					break;
				}
			}
		}
		if (intake_idx != std::numeric_limits<size_t>::max())
		{
			bool new_exceeded = talon_state.stator_current[intake_idx] > intake_current_threshold_;
            if (new_exceeded != current_exceeded_) {
                current_exceeded_ = new_exceeded;
                updated = true;
            }
		}
		else {
			ROS_ERROR_STREAM("candle_node : Can not find talon with name = " << "intake_leader");
		}
    }

    void path_callback(const geometry_msgs::TwistConstPtr &msg) {
        bool last_pathing = pathing;
        if (hypot(msg->linear.x, msg->linear.y) > 0.1 || msg->angular.z > 0.1) {
            pathing = true;
        } else {
            pathing = false;
        }
        if (last_pathing != pathing) {
            updated = true;
        }
    }
    
    void joystick_callback(const frc_msgs::JoystickState& msg) {
        ROS_INFO_STREAM_THROTTLE(2, "Joystick callback!");
        if(msg.directionLeftPress)
        {
            DPADLEFT = true;
            DPADRIGHT = false;
            DPADUP = false; 
            DPADDOWN = false;
            this->updated = true;
        }

        //Joystick1: directionRight
        if(msg.directionRightPress)
        {
            DPADRIGHT = true;
            DPADLEFT = false;
            DPADUP = false; 
            DPADDOWN = false;
            this->updated = true;
        }

        //Joystick1: directionUp
        if(msg.directionUpPress)
        {
            DPADUP = true;
            DPADLEFT = false;
            DPADRIGHT = false;
            DPADDOWN = false;
            this->updated = true;
        }

        //Joystick1: directionDown
        if(msg.directionDownPress)
        {
            DPADDOWN = true;
            DPADLEFT = false;
            DPADRIGHT = false;
            DPADUP = false; 
            this->updated = true;
        }
        if(msg.buttonXPress)
        {
            DPADDOWN = false;
            DPADLEFT = false;
            DPADRIGHT = false;
            DPADUP = false; 
            this->updated = true; 
        }
    }

    void button_box_callback(const frc_msgs::ButtonBoxState2023& msg) {
        if (msg.topMiddleConePress) {
            this->cone_button_pressed = !this->cone_button_pressed;
            ROS_INFO_STREAM("candle node : cone toggle");
            if (this->cone_button_pressed) {
                this->cube_button_pressed = false;
            }
            this->updated = true;
        }
        if (msg.topRightCubePress) {
            this->cube_button_pressed = !this->cube_button_pressed;
            ROS_INFO_STREAM("candle node : cube toggle");
            if (this->cube_button_pressed) {
                this->cone_button_pressed = false;
            }
            this->updated = true;
        }
    }

    void auto_mode_callback(const behavior_actions::AutoModeConstPtr& msg) {
        if (this->auto_mode != msg->auto_mode) {
            ROS_INFO_STREAM("new auto");
            this->auto_mode = msg->auto_mode;
            this->updated = true;
        }
    }

    double getYaw(const geometry_msgs::Quaternion &o) {
        tf2::Quaternion q;
        tf2::fromMsg(o, q);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        return y;
    }

    void imu_callback(const sensor_msgs::ImuConstPtr& msg) {
        double imu_angle = getYaw(msg->orientation);
        bool imu_zero_new = false;
        if (angles::shortest_angular_distance(imu_angle, M_PI) < imu_tolerance || angles::shortest_angular_distance(imu_angle, 0) < imu_tolerance) {
            imu_zero_new = true;
        }
        if (imu_zeroed != imu_zero_new) {
            ROS_INFO_STREAM("imu zero update");
            this->updated = true;
            this->imu_zeroed = imu_zero_new;
        }
    }

};

NodeCTX ctx;
// did not want to try and figure out the syntax for it in a struct, tried 
// ros::ServiceServer green_srv = node.advertiseService("/candle_node/set_leds_green", boost::bind(&NodeCTX::set_leds_green_callback, &ctx, _1));
// so just made it global
bool set_leds_green_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    // sets for three seconds, just to make sure operator (me) knows when they have been taking too long to place
    ctx.updated = true;
    ctx.OVERRIDE_GREEN = true;
    ctx.last_green_time = ros::Time::now();
    return true;
}

int main(int argc, char **argv) {
    // ROS node
    ros::init(argc, argv, "candle_node");
    ros::NodeHandle node;
    ros::Duration delay(1, 0);

    node.param<double>("/intaking/current_threshold", ctx.intake_current_threshold_, 100.0);

    // Team colour subscriber/callback
    ros::Subscriber button_box_subscriber = node.subscribe("/frcrobot_rio/button_box_states", 25, &NodeCTX::button_box_callback, &ctx);
    ros::Subscriber auto_mode_subscriber = node.subscribe("/auto/auto_mode", 1, &NodeCTX::auto_mode_callback, &ctx);
    ros::Subscriber imu_subscriber = node.subscribe("/imu/zeroed_imu", 1, &NodeCTX::imu_callback, &ctx);
    ros::Subscriber talon_state_subscriber = node.subscribe("/frcrobot_jetson/talonfxpro_states", 1, &NodeCTX::talon_callback, &ctx);
    ros::Subscriber path_follower_subscriber = node.subscribe("/path_follower/path_follower_pid/swerve_drive_controller/cmd_vel", 1, &NodeCTX::path_callback, &ctx);
    ros::Subscriber team_colour_subscriber = node.subscribe("/frcrobot_rio/match_data", 1, &NodeCTX::team_colour_callback, &ctx);
    ros::Subscriber joystick_state_sub = node.subscribe("/frcrobot_rio/joystick_states/1", 1, &NodeCTX::joystick_callback, &ctx);

    // ROS service clients (setting the CANdle)
    ros::ServiceClient colour_client = node.serviceClient<candle_controller_msgs::Colour>("/frcrobot_jetson/candle_controller/colour", false);
    ros::ServiceClient animation_client = node.serviceClient<candle_controller_msgs::Animation>("/frcrobot_jetson/candle_controller/animation", false);
    ros::ServiceClient brightness_client = node.serviceClient<candle_controller_msgs::Brightness>("/frcrobot_jetson/candle_controller/brightness", false);
    ros::ServiceServer green_srv = node.advertiseService("/candle_node/set_leds_green", &set_leds_green_callback);
    // param = /intaking/current_threshold


    // Wait for the CANdle services to start
    colour_client.waitForExistence();
    animation_client.waitForExistence();
    brightness_client.waitForExistence();

    // Clear the CANdle LEDs
    candle_controller_msgs::Colour clear_req;
    clear_req.request.start = 0;
    clear_req.request.count = MAX_LED;
    while (!colour_client.call(clear_req)) {
        delay.sleep();
    }
    candle_controller_msgs::Brightness brightness_req;
    brightness_req.request.brightness = 1.0;
    while (!brightness_client.call(brightness_req)) {
        delay.sleep();
    }
    ROS_INFO_STREAM("Candle node running; cleared LEDs.");
    delay.sleep();

    // ROS loop
    ros::Rate r(10);
    // Candle node pregame is getting replaced
    while (ctx.disabled) {
        r.sleep();
    }
    while (ros::ok()) {
        ros::spinOnce();
        if (ctx.disabled && ctx.updated) {
            candle_controller_msgs::Colour colour_req;
            colour_req.request.start = 0;
            colour_req.request.count = MAX_LED;

            candle_controller_msgs::Colour imu_colour_req;
            imu_colour_req.request.start = 0;
            imu_colour_req.request.count = 4;
            imu_colour_req.request.red = ctx.imu_zeroed ? 0 : 128;
            imu_colour_req.request.green = ctx.imu_zeroed ? 128 : 0;
            imu_colour_req.request.blue = 0;
            if (ctx.DPADDOWN) {
                // down 
                ros::Duration(0.25).sleep();

                if (colour_client.call(imu_colour_req)) {
                    ROS_INFO_STREAM("Updated LEDs imu zero");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }

                candle_controller_msgs::Animation animation_req;
                animation_req.request.animation_type = animation_req.request.ANIMATION_TYPE_RAINBOW;
                animation_req.request.brightness = 1.0;
                animation_req.request.speed = 1.0;
                animation_req.request.start = 0;
                animation_req.request.count = MAX_LED;
                if (animation_client.call(animation_req)) {
                    ROS_INFO_STREAM("Updated LEDs rainbow");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }
            }
            else if (ctx.DPADUP) {
                // down 
                ros::Duration(0.25).sleep();

                if (colour_client.call(imu_colour_req)) {
                    ROS_INFO_STREAM("Updated LEDs imu zero");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }

                candle_controller_msgs::Animation animation_req;
                animation_req.request.animation_type = animation_req.request.ANIMATION_TYPE_LARSON;
                animation_req.request.brightness = 1.0;
                animation_req.request.red = 255;
                animation_req.request.blue = 255;
                animation_req.request.speed = 0.5;
                animation_req.request.start = 0;
                animation_req.request.count = MAX_LED;
                animation_req.request.param5 = 2.0; // Number of LEDs lit
                if (animation_client.call(animation_req)) {
                    ROS_INFO_STREAM("Updated LEDs rainbow");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }
            }
            else if (ctx.DPADRIGHT) {

            }
            else if (ctx.DPADLEFT) {

            }
            else if (ctx.auto_mode <= 3) {
                // up (placing autos)
                ros::Duration(0.25).sleep();

                if (colour_client.call(imu_colour_req)) {
                    ROS_INFO_STREAM("Updated LEDs imu zero");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }

                candle_controller_msgs::Animation animation_req;
                animation_req.request.animation_type = animation_req.request.ANIMATION_TYPE_RAINBOW;
                animation_req.request.brightness = 1.0;
                animation_req.request.speed = 0.5;
                animation_req.request.start = 0;
                animation_req.request.count = MAX_LED;
                if (animation_client.call(animation_req)) {
                    ROS_INFO_STREAM("Updated LEDs rainbow");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }
            }
            else if (ctx.auto_mode <= 6) {
                // middle (engage/drive back)
                colour_req.request.red = 128;
                colour_req.request.green = 128;
                colour_req.request.blue = 128;
                if (colour_client.call(colour_req)) {
                    ROS_INFO_STREAM("Updated LEDs white");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }

                ros::Duration(0.25).sleep();

                if (colour_client.call(imu_colour_req)) {
                    ROS_INFO_STREAM("Updated LEDs imu zero");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }
            }
            else {
                // no-op
                colour_req.request.red = 255;
                colour_req.request.green = 128;
                colour_req.request.blue = 0;
                if (colour_client.call(colour_req)) {
                    ROS_INFO_STREAM("Updated LEDs orange");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }

                ros::Duration(0.25).sleep();

                if (colour_client.call(imu_colour_req)) {
                    ROS_INFO_STREAM("Updated LEDs imu zero");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }
            }
        }
        else if (ctx.updated) {
            candle_controller_msgs::Colour colour_req;
            colour_req.request.start = 0;
            colour_req.request.count = MAX_LED;

            if (ctx.OVERRIDE_GREEN) {
                colour_req.request.green = 255;
            } else if (ctx.current_exceeded_) {
                ros::Duration(0.1).sleep();
                colour_req.request.green = 255;
                if (colour_client.call(colour_req)) {
                    ROS_INFO_STREAM("Updated LEDs");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }
                ros::Duration(1.0).sleep();
                colour_req.request.green = 0;
            } else if (ctx.cone_button_pressed) {
                // Yellow colour
                colour_req.request.red = 255;
                colour_req.request.green = 150;
            } else if (ctx.cube_button_pressed) {
                // Purple colour
                colour_req.request.red = 150;
                colour_req.request.blue = 255;
            } else if (ctx.pathing) {
                if (colour_client.call(colour_req)) {
                    ROS_INFO_STREAM("Updated LEDs");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }
                ros::Duration(0.25).sleep();
                candle_controller_msgs::Animation animation_req;
                animation_req.request.animation_type = animation_req.request.ANIMATION_TYPE_RAINBOW;
                animation_req.request.brightness = 1.0;
                animation_req.request.speed = 1.0;
                animation_req.request.start = 0;
                animation_req.request.count = MAX_LED;
                if (animation_client.call(animation_req)) {
                    ROS_INFO_STREAM("Updated LEDs rainbow");
                    ctx.updated = false;
                } else {
                    ROS_ERROR_STREAM("Failed to update LEDs");
                }
                continue;
            }

            if (colour_client.call(colour_req)) {
                ROS_INFO_STREAM("Updated LEDs, R=" << std::to_string(colour_req.request.red) << " G=" << std::to_string(colour_req.request.green) << " B=" << std::to_string(colour_req.request.blue));
                ctx.updated = false;
            } else {
                ROS_ERROR_STREAM("Failed to update LEDs");
            }
        }
        r.sleep();
        if (ctx.OVERRIDE_GREEN && (ros::Time::now() - ctx.last_green_time) >= ros::Duration(3)) {
            ctx.OVERRIDE_GREEN = false;
            ctx.updated = true;
        }
    }
    return 0;
}
