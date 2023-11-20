#include <ros/ros.h>
#include <string>
#include "field_obj/Detection.h"
#include "field_obj/Object.h"
#include "frc_msgs/MatchSpecificData.h"

ros::Publisher pub;

// default to red, in case match data isn't coming through (e.g. we are practicing)
std::string friendlyCargo = "red_cargo";
std::string opponentCargo = "blue_cargo";
bool allianceKnown = true;

void detectionCallback(const field_obj::DetectionConstPtr objDetectionMsg) {
    field_obj::Detection copy = *objDetectionMsg;

    for (field_obj::Object &obj : copy.objects) {
        // Don't filter anything if alliances are unknown
        // Cargo will remain labeled as blue or red if the alliance color is unknown
        if (!allianceKnown) {
            continue;
        }

        if (obj.id == friendlyCargo) {
            obj.id = "friendly_cargo";
        } else if (obj.id == opponentCargo) {
            obj.id = "opponent_cargo";
        }
    }
    pub.publish(copy);
}

void matchCallback(const frc_msgs::MatchSpecificDataConstPtr matchDataMsg) {
    const std::string redCargo = "red_cargo";
    const std::string blueCargo = "blue_cargo";

    switch(matchDataMsg->allianceColor) {
        case frc_msgs::MatchSpecificData::ALLIANCE_COLOR_RED:
            friendlyCargo = redCargo;
            opponentCargo = blueCargo;
            allianceKnown = true;
            break;
        case frc_msgs::MatchSpecificData::ALLIANCE_COLOR_BLUE:
            friendlyCargo = blueCargo;
            opponentCargo = redCargo;
            allianceKnown = true;
            break;
        default:
            break; 
            /*ROS_ERROR_STREAM("Invalid alliance color in MatchSpecificData msg: "
                             << matchDataMsg->allianceColor);
            allianceKnown = false;
            */
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "cargo_detection_filter");
    ros::NodeHandle nh;

    pub = nh.advertise<field_obj::Detection>("object_detection_world_filtered", 2);
    ros::Subscriber detections = nh.subscribe("object_detection_world", 2, detectionCallback);
    ros::Subscriber matchData = nh.subscribe("/frcrobot_rio/match_data", 2, matchCallback);

    ros::spin();

    return 0;
}
