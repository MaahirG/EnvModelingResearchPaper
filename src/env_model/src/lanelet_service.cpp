#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <experimental/filesystem>
#include <unordered_map>

#include "autoware_lanelet2_msgs/MapBin.h"
#include "lanelet2_extension/utility/message_conversion.h"
#include "path_planning_msgs/Lanelet2MapService.h"
#include "ros/ros.h"

using namespace lanelet;
using namespace path_planning_msgs;

std::string MAPS_DIR;
std::unordered_map<std::string, projection::UtmProjector> UTM_PROJECTORS_MAP = {
    {"Town05", projection::UtmProjector(Origin({0.0, 0.0}))},
    {"mcity", projection::UtmProjector(Origin(GPSPoint{42.2998, -83.6989}))}};

std::string get_map_name(Lanelet2MapService::Request &req) {
  std::string carla_town;
  if (req.use_map_name.data) {
    return req.map_name.data;
  } else if (ros::param::get("/carla/town", carla_town)) {
    return carla_town;
  }
  return "";
}

bool get_map(Lanelet2MapService::Request &req,
             Lanelet2MapService::Response &res) {
  std::string map_name = get_map_name(req);
  if (map_name.empty()) {
    ROS_ERROR("Unable to determine Map to serve, please specify one");
    return false;
  }
  std::string map_file = MAPS_DIR + "/osm/" + map_name + ".osm";
  if (!std::experimental::filesystem::exists(map_file)) {
    ROS_ERROR_STREAM(
        "Map "
        << map_file.c_str()
        << " does not exist. Please add it to "
           "https://git.uwaterloo.ca/WATonomous/map_data/-/tree/master/osm");
    return false;
  }
  projection::UtmProjector projector(Origin({0, 0}));
  if (req.use_origin.data) {
    projector = projection::UtmProjector(
        Origin({req.utm_origin.latitude, req.utm_origin.longitude}));
  } else if (UTM_PROJECTORS_MAP.find(map_name) != UTM_PROJECTORS_MAP.end()) {
    projector = UTM_PROJECTORS_MAP.at(map_name);
  } else {
    ROS_ERROR("Unable to determine UTM Projector to use, please specify one");
    return false;
  }

  try {
    lanelet::LaneletMapPtr map = load(map_file, projector);
    autoware_lanelet2_msgs::MapBin map_msg;
    lanelet::utils::conversion::toBinMsg(map, &map_msg);
    res.map_bin = map_msg;
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("Failed to load and convert map, error: " << e.what());
    return false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lanelet2_map_service");
  ros::NodeHandle n;
  if (!n.getParam("MAPS_DIR", MAPS_DIR)) {
    ROS_ERROR("MAPS_DIR param required");
    return 1;
  }
  if (!std::experimental::filesystem::exists(MAPS_DIR)) {
    ROS_ERROR_STREAM("MAPS_DIR " << MAPS_DIR << " does not exist");
    return 1;
  }

  ros::ServiceServer map_service =
      n.advertiseService("/lanelet2_map_service", get_map);

  ros::spin();

  return 0;
}
