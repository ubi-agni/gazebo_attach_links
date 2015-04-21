#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <agni_gazebo_attach/AttachLinks.h>

namespace gazebo

{
class AgniGazeboAttach : public WorldPlugin
{
public:

  
  
private:  
  ros::NodeHandle* rosnode_;
  // Pointer to the model
  physics::WorldPtr world;
  physics::PhysicsEnginePtr physics;
  bool attached;
  ros::Publisher pub;
  ros::Subscriber sub;
  int count;
  event::ConnectionPtr updateConnection;
  
  physics::ModelPtr ref_model,tgt_model;
  physics::LinkPtr ref_link,tgt_link;
  math::Pose relative_pose;
  
public:
  AgniGazeboAttach() : WorldPlugin(),  rosnode_(NULL), attached(false), count(0)
  {
    
  }
  
  ~AgniGazeboAttach()
  {
    delete rosnode_;
  }
  
  void AttachCB(const agni_gazebo_attach::AttachLinksConstPtr& msg)
  {
    if(!attached)
    {
      if(msg->operation == msg->ATTACH)
        attached=Attach(msg->reference_model_name,msg->reference_link_name,msg->target_model_name,msg->target_link_name);
      else
        ROS_INFO("Nothing to detach");
    }
    else
    {
      if(msg->operation == msg->DETACH)
        attached=false;
      else
        ROS_WARN("link already attached, detach first");
    }
  }
  

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // Get the world name.
    world = _world;
    
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO("Hello World!");
    rosnode_ = new ros::NodeHandle();
    
    pub = rosnode_->advertise<std_msgs::Bool>("gazebo_attached", 10);
    sub = rosnode_->subscribe("gazebo_attach", 10, &AgniGazeboAttach::AttachCB, this);
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&AgniGazeboAttach::OnUpdate, this, _1));
    
  }
  
  void OnUpdate(const common::UpdateInfo & )
  {
    std_msgs::Bool msg;
    if (count>1000)
    {
      count=0;
      msg.data = attached;
      pub.publish(msg);
      ros::spinOnce();
    }
    count++;
    if (attached)
	  {
      math::Pose pose = relative_pose + ref_link->GetWorldPose();
	    tgt_link->SetWorldPose(pose);
    }    
  }


  bool Attach(const std::string ref_name, const std::string ref_link_name,
              const std::string tgt_name, const std::string tgt_link_name)
  {
    // get the models    
    ref_model = world->GetModel(ref_name);
    if(!ref_model)
    {
      ROS_FATAL_STREAM("Cannot find model "<<ref_name);
      return false;
    }
    
    tgt_model = world->GetModel(tgt_name);
    if(!tgt_model)
    {
      ROS_FATAL_STREAM("Cannot find model "<<tgt_name);
      return false;
    }
    
    // get the links
    ref_link = ref_model->GetLink(ref_link_name);
    if(!ref_link)
    {
      ROS_FATAL_STREAM("Cannot find link "<<ref_link_name<<" in model "<<ref_name);
      return false;
    }
    
    tgt_link = tgt_model->GetLink(tgt_link_name);
    if(!tgt_link)
    {
      ROS_FATAL_STREAM("Cannot find link "<<tgt_link_name<<" in model "<<tgt_name);
      return false;
    }
    
    // compute the relative pose
    relative_pose = tgt_link->GetWorldPose() - ref_link->GetWorldPose();
    return true;
    
  }


};




GZ_REGISTER_WORLD_PLUGIN(AgniGazeboAttach)
}
