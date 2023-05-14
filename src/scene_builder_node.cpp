#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <memory>
#include <std_srvs/Trigger.h>

#include <ros/callback_queue.h>

void addBOX(moveit::planning_interface::PlanningSceneInterface &planning_scene,
            double dim_x, double dim_y, double dim_z,
            geometry_msgs::PoseStamped pose, const std::string &obj_id) {
  std::cout << "\n#####\nAdding box [" << dim_x << ", " << dim_y << ", "
            << dim_z << "]\n pose:\n"
            << pose << "\n#####\n";

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = pose.header.frame_id;
  collision_object.id = obj_id;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = dim_x;
  primitive.dimensions[primitive.BOX_Y] = dim_y;
  primitive.dimensions[primitive.BOX_Z] = dim_z;

  pose.pose.position.z += dim_z / 2.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose.pose);
  collision_object.operation = collision_object.ADD;

  planning_scene.applyCollisionObject(collision_object);
  ros::Duration(1).sleep();
}

std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>
    planning_scene_interface;

std::unique_ptr<moveit::planning_interface::MoveGroupInterface>
    move_group_interface;

void clearScene() {
  // RIMUOVO TUTTO GLI OGGETTI PRESENTI
  for (auto const &element : planning_scene_interface->getAttachedObjects()) {
    move_group_interface->detachObject(element.first);
    ros::Duration(1).sleep();
  }
  auto objects_map = planning_scene_interface->getObjects();
  std::vector<std::string> obj_keys;
  for (auto const &element : objects_map) {
    obj_keys.push_back(element.first);
  }
  planning_scene_interface->removeCollisionObjects(obj_keys);
  ros::Duration(1.0).sleep();
}

void buildScene() {

  clearScene();

  // COSTRUISCO LA SCENA

  double x1 = 0.5;
  double y1 = 0.60;
  double DX_0 = 0.2;
  double Dy_f = 0.25;
  double Table_DX = 0.5;
  double Table_DY = 1.0;
  double Table_DZ = 0.4;
  double DX = 0.5;
  double DX_2 = 0.25;
  double DZ_2 = 0.5;
  double DZ_3 = 0.25;

  // box 1
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = x1 + Table_DX / 2.0;
    addBOX(*planning_scene_interface, Table_DX, Table_DY, Table_DZ, pose,
           "obst_1");
  }

  // box 2
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.orientation.w = 1.0;
    pose.pose.position.y = y1 + Table_DX / 2.0;
    addBOX(*planning_scene_interface, Table_DY, Table_DX, Table_DZ, pose,
           "obst_2");
  }

  // box 3
  {

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = DX / 2.0 + DX_2 / 2.0;
    pose.pose.position.y = y1 + Table_DX / 2.0;
    pose.pose.position.z = Table_DZ;
    addBOX(*planning_scene_interface, DX_2, Table_DX, DZ_2, pose, "obst_3");
  }

  // box 4
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = -DX / 2.0 - DX_2 / 2.0;
    pose.pose.position.y = y1 + Table_DX / 2.0;
    pose.pose.position.z = Table_DZ;
    addBOX(*planning_scene_interface, DX_2, Table_DX, DZ_2, pose, "obst_4");
  }

  // box 5
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = y1 + Table_DX / 2.0;
    pose.pose.position.z = Table_DZ + DZ_2;
    addBOX(*planning_scene_interface, Table_DY, Table_DX, DZ_3, pose, "obst_5");
  }
}

bool build_scene_srv_cb(std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response &res) {

  buildScene();

  res.success = true;

  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "scene_builder");
  ros::NodeHandle node_handle;

  static const std::string PLANNING_GROUP = "panda_arm";

  move_group_interface =
      std::unique_ptr<moveit::planning_interface::MoveGroupInterface>(
          new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));

  planning_scene_interface =
      std::unique_ptr<moveit::planning_interface::PlanningSceneInterface>(
          new moveit::planning_interface::PlanningSceneInterface);

  // ros::NodeHandle node_handle_srv;
  // node_handle_srv.setCallbackQueue(callbk_q);

  ros::ServiceServer service =
      node_handle.advertiseService("build_scene", build_scene_srv_cb);

  ros::spin();

  return 0;
}
