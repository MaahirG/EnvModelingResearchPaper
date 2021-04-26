#include <geometry_msgs/TransformStamped.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Optional.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/GenericTrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <boost/range/combine.hpp>
#include <memory>
#include <mutex>
#include <unordered_set>

#include "common_msgs/Obstacle.h"
#include "common_msgs/TrackedObstacleList.h"
#include "lanelet2_extension/utility/message_conversion.h"
#include "path_planning_msgs/DestinationList.h"
#include "path_planning_msgs/GetBehavior.h"
#include "path_planning_msgs/Lanelet2MapService.h"
#include "path_planning_msgs/LaneletDestinationService.h"
#include "path_planning_msgs/RegulatoryElementService.h"
#include "pp_common/ped_reg_elem.hpp"

using namespace lanelet;
using namespace path_planning;
using namespace path_planning_msgs;
using namespace std;

using LaneletSet = unordered_set<Lanelet>;

lanelet::LaneletMapPtr lanelet_map = std::make_shared<lanelet::LaneletMap>();
routing::RoutingGraphUPtr routingGraph;
std::unordered_map<uint32_t, Id> tracker_id_to_reg_elem_id_map;
routing::LaneletPath curr_path;
Optional<Point2d> dest_point;
std::mutex map_mutex;

ros::ServiceClient lanelet_destination_service;

template <typename T>
std::ostream &operator<<(std::ostream &output, std::vector<T> const &values) {
  for (auto const &value : values) {
    output << value << ", ";
  }
  return output;
}

void destination_list_callback(DestinationList dest_list) {
  if (dest_list.destination_list.empty()) {
    ROS_WARN("Routing Failed: Empty Destination List");
    return;
  }
  // *** Get destination lanelet(s) ***
  path_planning_msgs::LaneletDestinationService destination_call;
  // TODO: Implement multiple destination routing
  destination_call.request.destination_address.data =
      dest_list.destination_list[0];
  if (!lanelet_destination_service.call(destination_call) ||
      destination_call.response.destination_lanelets.empty()) {
    ROS_WARN_STREAM(
        "Routing Failed: Destination lanelet service failed for destination "
        "address: "
        << destination_call.request.destination_address.data);
    return;
  }

  // *** Get current lanelet(s) ***
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped base_link_map_tf;
  try {
    base_link_map_tf = tfBuffer.lookupTransform(
        "map", "base_link", ros::Time(0), ros::Duration(5.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("Routing Failed: %s", ex.what());
    return;
  }
  Point2d query_point{utils::getId(), base_link_map_tf.transform.translation.x,
                      base_link_map_tf.transform.translation.y};
  auto curr_lls =
      geometry::findWithin2d(lanelet_map->laneletLayer, query_point);
  if (curr_lls.size() == 0) {
    ROS_WARN_STREAM(
        "Routing Failed: Failed to localize ego vehicle within map, please "
        "move onto a "
        "lanelet. Query point: "
        << query_point);
    return;
  }

  // *** Find the best way to the destination(s) ***
  Optional<routing::LaneletPath> shortest_path;
  double shortest_path_length;
  for (auto start : curr_lls) {
    for (auto end_id : destination_call.response.destination_lanelets) {
      auto end_ll = *lanelet_map->laneletLayer.find(end_id);
      auto path = routingGraph->shortestPath(start.second, end_ll);
      if (!path) {
        continue;
      }
      double path_length = 0;
      for (const auto &ll : *path) {
        path_length += geometry::length2d(ll);
      }
      if (!shortest_path || path_length < shortest_path_length) {
        shortest_path = path;
        shortest_path_length = path_length;
      }
    }
  }
  if (!shortest_path) {
    std::vector<Id> start_ids;
    for (const auto &ll : curr_lls) {
      start_ids.push_back(ll.second.id());
    }
    ROS_WARN_STREAM("Routing Failed: Unable to find path from start ["
                    << start_ids << "] to dest set ["
                    << destination_call.response.destination_lanelets << "]");
    return;
  }
  curr_path = shortest_path.get();
  dest_point =
      Point2d{utils::getId(), destination_call.response.destination_point.x,
              destination_call.response.destination_point.y};

  ROS_INFO_STREAM("Found path to destination with " << curr_path.size()
                                                    << " lanelets");
}

LineString3d obstacle_states_to_ls(
    std::vector<common_msgs::TrackedObstacleState> states) {
  LineString3d ls{InvalId, {}};
  for (auto state : states) {
    ls.push_back(
        Point3d(InvalId, state.pose.position.x, state.pose.position.y));
  }
  return ls;
}

LaneletSet get_pedestrian_movement_conflicts(common_msgs::TrackedObstacle ped) {
  LaneletSet conflict_lls;
  for (const auto &ll : geometry::findWithin2d(
           lanelet_map->laneletLayer,
           utils::to2D(obstacle_states_to_ls(ped.predicted_states)))) {
    conflict_lls.insert(ll.second);
  }
  return conflict_lls;
}

// Calculates how many seconds it's been since a tracked obstacle has moved 0.5
// meters from where it currently is If we've never seen it move 0.5 meters,
// returns the time that the track has been alive
ros::Duration history_static_time(
    std::vector<common_msgs::TrackedObstacleState> history) {
  ros::Time most_recent_stamp = history.back().header.stamp;
  ros::Time most_recent_move_stamp(0, 0);
  // We want to go from most recent to oldest, which means going through the
  // history array from back to front
  for (auto state_it = history.rbegin(); state_it != history.rend();
       ++state_it) {
    if (hypot(state_it->pose.position.x - history.back().pose.position.x,
              state_it->pose.position.y - history.back().pose.position.y) >
        0.5) {
      most_recent_move_stamp = state_it->header.stamp;
      break;
    }
  }
  if (most_recent_move_stamp.isZero()) {
    return history.back().header.stamp - history.front().header.stamp;
  }
  return most_recent_stamp - most_recent_move_stamp;
}

LaneletSet get_pedestrian_waiting_conflicts(common_msgs::TrackedObstacle ped) {
  ros::Duration alive_time = ped.observation_history.back().header.stamp -
                             ped.observation_history.front().header.stamp;
  ros::Duration waiting_time = history_static_time(ped.observation_history);
  // If it's moved in the past 1 seconds it's not waiting
  // If it's been waiting for more than 10 seconds it's not gonna cross
  if ((alive_time.toSec() >= 1 && waiting_time.toSec() <= 1) ||
      waiting_time.toSec() > 10) {
    return {};
  }
  LaneletSet conflict_lls;
  // A waiting pedestrian conflicts with all lanelets in a 5 meter radius around
  // it
  for (const auto &ll :
       geometry::findWithin2d(lanelet_map->laneletLayer,
                              BasicPoint2d(ped.obstacle.pose.pose.position.x,
                                           ped.obstacle.pose.pose.position.y),
                              5)) {
    conflict_lls.insert(ll.second);
  }
  return conflict_lls;
}

Lanelets get_pedestrian_conflicts(common_msgs::TrackedObstacle ped) {
  auto waiting_conflicts = get_pedestrian_waiting_conflicts(ped);
  auto moving_conflicts = get_pedestrian_movement_conflicts(ped);
  waiting_conflicts.insert(moving_conflicts.begin(), moving_conflicts.end());
  Lanelets ret;
  ret.insert(ret.end(), waiting_conflicts.begin(), waiting_conflicts.end());
  return ret;
}

void tracked_obstacles_callback(
    common_msgs::TrackedObstacleList tracked_obs_list) {
  const std::lock_guard<std::mutex> ego_lock(map_mutex);
  for (auto tracked_obs : tracked_obs_list.tracked_obstacles) {
    if (tracked_obs.obstacle.label != common_msgs::Obstacle::OBS_TP_PED) {
      continue;
    }
    auto conflict_lls = get_pedestrian_conflicts(tracked_obs);
    auto predicted_ped_states =
        obstacle_states_to_ls(tracked_obs.predicted_states);
    // If we already have this ped in the map, update its conflicts
    if (tracker_id_to_reg_elem_id_map.find(tracked_obs.obstacle.object_id) !=
        tracker_id_to_reg_elem_id_map.end()) {
      // Get ped already in map by its ID
      RegulatoryElementPtr ped = lanelet_map->regulatoryElementLayer.get(
          tracker_id_to_reg_elem_id_map[tracked_obs.obstacle.object_id]);
      std::dynamic_pointer_cast<env_model::PedRegElem>(ped)
          ->updatePredictedStates(predicted_ped_states);
      // Find lanelets that own the ped
      for (auto &ll : lanelet_map->laneletLayer) {
        auto peds = ll.regulatoryElementsAs<env_model::PedRegElem>();
        bool owns_ped = std::find(peds.begin(), peds.end(), ped) != peds.end();
        bool should_own_ped =
            std::find(conflict_lls.begin(), conflict_lls.end(), ll) !=
            conflict_lls.end();
        if (owns_ped && !should_own_ped) {
          ll.removeRegulatoryElement(ped);
        } else if (!owns_ped && should_own_ped) {
          ll.addRegulatoryElement(ped);
        }
      }
    }
    // Else create reg elem for ped and add to map
    else {
      Id reg_elem_id = utils::getId();
      tracker_id_to_reg_elem_id_map[tracked_obs.obstacle.object_id] =
          reg_elem_id;
      RuleParameterMap params{
          {RoleNameString::RefLine, {predicted_ped_states}}};
      RegulatoryElementPtr ped =
          RegulatoryElementFactory::create("pedestrian", reg_elem_id, params);
      lanelet_map->add(ped);
      for (auto &ll : conflict_lls) {
        ll.addRegulatoryElement(ped);
      }
    }
  }
}

void prune_dead_tracks_callback(const ros::TimerEvent &) {
  for (auto &ll : lanelet_map->laneletLayer) {
    auto peds = ll.regulatoryElementsAs<env_model::PedRegElem>();
    if (peds.empty()) {
      continue;
    }
    for (auto &ped : peds) {
      if (ped->isTrackDead()) {
        ll.removeRegulatoryElement(ped);
      }
    }
  }
}

bool get_regulatory_elements(RegulatoryElementService::Request &req,
                             RegulatoryElementService::Response &res) {
  Lanelets lls;
  for (const auto &ll : lanelet_map->laneletLayer) {
    if (!ll.regulatoryElements().empty()) {
      lls.push_back(ll);
    }
  }
  lanelet::utils::conversion::toBinMsg(utils::createMap(lls), &res.map_bin);
  return true;
}

bool lane_change(const ConstLanelet &from, const ConstLanelet &to) {
  auto besides_from = routingGraph->besides(from);
  return std::find(besides_from.begin(), besides_from.end(), to) !=
         besides_from.end();
}

// The distance computed is to the closest point on the ll's centerline that is
// before the stopping point. Therefore the distance is always an under
// approximation of the true stopping distance, and the most accurate under
// approximation that ends at a point on the centerline. The stopping point is
// perfectly accurate no matter the sparsity of the ll's centerline points.
pair<double, lanelet::BasicPoint2d> get_stopping_dist_and_point(
    const ConstLanelet &ll) {
  double stopping_dist = numeric_limits<double>::max();
  lanelet::BasicPoint2d stopping_point;
  for (auto ped_reg : ll.regulatoryElementsAs<env_model::PedRegElem>()) {
    // We need to stop for this boi, but where???

    // Hybrid type needed to call into Boost geometry functions
    auto hybrid_ll_center = traits::toHybrid(ll.centerline2d());
    auto hybrid_ped_line =
        traits::toHybrid(utils::to2D(ped_reg->getPredictedStates()));
    auto ped_point = utils::to2D(ped_reg->getPredictedStates()[0]);

    vector<lanelet::BasicPoint2d> intersection_points;
    // Put intersection point of ped with lanelet centerline into
    // stopping_point
    boost::geometry::intersection(hybrid_ll_center, hybrid_ped_line,
                                  intersection_points);
    ROS_WARN_COND(intersection_points.size() >= 2,
                  "Pedestrian trajectory intersects with lanelet centerline "
                  "more than once");

    lanelet::BasicPoint2d ped_stopping_point;
    // Stopping point is either the predicted intersection, or if no predicted
    // intersection, the closest point
    if (intersection_points.empty()) {
      ped_stopping_point = geometry::project(ll.centerline2d(), ped_point);
    } else {
      ped_stopping_point = intersection_points[0];
    }

    auto ped_stopping_point_seg = geometry::closestSegment(
        ll.centerline2d().basicLineString(), ped_stopping_point);
    double ped_stopping_point_dist =
        geometry::toArcCoordinates(hybrid_ll_center,
                                   ped_stopping_point_seg.first)
            .length;
    if (ped_stopping_point_dist < stopping_dist) {
      stopping_point = ped_stopping_point;
      stopping_dist = ped_stopping_point_dist;
    }
  }

  return make_pair(stopping_dist, stopping_point);
}

bool get_behavior(GetBehavior::Request &req, GetBehavior::Response &res) {
  const std::lock_guard<std::mutex> ego_lock(map_mutex);
  for (unsigned int ll_i = 0; ll_i < curr_path.size(); ll_i++) {
    auto &ll = curr_path[ll_i];
    if (ll_i < curr_path.size() - 1 && lane_change(ll, curr_path[ll_i + 1])) {
      continue;
    }
    bool ll_is_last = true;
    Optional<ConstPoint2d> closest_point_on_last_lanelet;
    for (unsigned int ll_next_i = ll_i + 1; ll_next_i < curr_path.size();
         ll_next_i++) {
      if (!lane_change(ll, curr_path[ll_next_i])) {
        ll_is_last = false;
        break;
      }
    }
    if (ll_is_last) {
      double min_dist =
          geometry::distance2d(*ll.centerline2d().begin(), dest_point.get());
      closest_point_on_last_lanelet = *ll.centerline2d().begin();
      for (const auto &point : ll.centerline2d()) {
        double dist = geometry::distance2d(point, dest_point.get());
        if (dist < min_dist) {
          min_dist = dist;
          closest_point_on_last_lanelet = point;
        }
      }
    }
    pair<double, BasicPoint2d> stop = get_stopping_dist_and_point(ll);
    auto len = geometry::length2d(ll);

    // Accumulating length fractions of line segments along center polyline
    auto fracs = geometry::accumulatedLengthRatios(ll.centerline2d());
    for (unsigned int i = 0; i < ll.centerline2d().size(); i++) {
      auto point = ll.centerline2d()[i];
      // Fracs start at end of first segment, off by 1 w.r.t. point
      double frac = i == 0 ? 0 : fracs[i - 1];

      geometry_msgs::Point lp;
      // If we have a stopping distance on the curr lanelet and adding the next
      // point would overpass that distance,
      //   instead just add the stopping point and return
      if (frac * len >= stop.first) {
        lp.x = stop.second.x();
        lp.y = stop.second.y();
        res.frenet_reference_line.push_back(lp);
        return true;
      }
      lp.x = point.x();
      lp.y = point.y();
      res.frenet_reference_line.push_back(lp);
      if (!!closest_point_on_last_lanelet &&
          point == closest_point_on_last_lanelet.get()) {
        return true;
      }
    }
  }
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "env_model");
  ros::NodeHandle n;

  RegisterRegulatoryElement<env_model::PedRegElem> ped_reg;

  // *** Initialize Lanelet Map and Routing Graph ***
  auto lanelet2_map_service =
      n.serviceClient<path_planning_msgs::Lanelet2MapService>(
          "/lanelet2_map_service");
  path_planning_msgs::Lanelet2MapService map_call;
  while (ros::ok() && !lanelet2_map_service.call(map_call)) {
    ros::Duration(1).sleep();
  }
  if (!ros::ok()) {
    return 0;
  }
  lanelet::utils::conversion::fromBinMsg(map_call.response.map_bin,
                                         lanelet_map);
  traffic_rules::TrafficRulesPtr trafficRules =
      traffic_rules::TrafficRulesFactory::create(Locations::Germany,
                                                 Participants::Vehicle);
  routingGraph = routing::RoutingGraph::build(*lanelet_map, *trafficRules);
  ROS_INFO_STREAM("Lanelet map and routing graph initialized with "
                  << lanelet_map->laneletLayer.size() << " lanelets.");
  for (auto err : routingGraph->checkValidity()) {
    ROS_WARN_STREAM("Routing Graph Error: " << err);
  }

  ros::Subscriber tracked_obstacle_subscriber =
      n.subscribe("/tracked_obstacles", 1, tracked_obstacles_callback);
  // Prune dead tracks at 1HZ
  ros::Timer timer =
      n.createTimer(ros::Duration(1.f), prune_dead_tracks_callback);
  ros::Subscriber destination_list_sub = n.subscribe(
      "/path_planning/destination_list", 1, destination_list_callback);

  lanelet_destination_service =
      n.serviceClient<path_planning_msgs::LaneletDestinationService>(
          "/lanelet_destination_service");

  ros::ServiceServer bp_service =
      n.advertiseService("/get_behavior", get_behavior);
  ros::ServiceServer reg_elem_service =
      n.advertiseService("/env_model_reg_elems", get_regulatory_elements);
  ros::spin();

  return 0;
}
