/**
 * @file   agni_grasp_force_limiter.cpp
 * @author Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
 * @date   Jan 2014
 *
 * @brief  executes hand postures through the HandCommander but also sets maximum force and velocity
 *
 */

#include <agni_grasp_force_limiter/agni_grasp_force_limiter.hpp>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <std_msgs/Float64.h>

#define CTRLTYPE	"sr_mechanism_controllers/SrhMixedPositionVelocityJointController"
#define CUTOUT_FORCE_LIMIT 	250
#define	CUTOUT_FORCE_COUNT	100
#define MONITOR_FORCE_LIMIT 	150
#define	MONITOR_FORCE_COUNT		100
#define LEAST_ACCEPTED_ERR_REDUCTION	0.0017

AgniGraspForceLimiter::AgniGraspForceLimiter() : nh_tilde("~")
{
  
  //We use the pr2_controller_manager/list_controllers service to detect get the controllers name and be able to query their setGains services
    if(ros::service::waitForService("pr2_controller_manager/list_controllers", ros::Duration(3.0)))
    {
      // initialize the setGains services
      initializeServices();
      
      sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, &AgniGraspForceLimiter::stateProcessingCB, this);
      ROS_INFO("Agni Grasp Force Limiter ready");
    }
    else
      ROS_ERROR("Agni Grasp Force Limiter did not find controllers");
}

AgniGraspForceLimiter::~AgniGraspForceLimiter()
{
}

// each time a joint_state is received process it
void AgniGraspForceLimiter::stateProcessingCB(const sensor_msgs::JointStateConstPtr& msg)
{	
  ROS_DEBUG("got a message");
  
  std::map<std::string,ros::Publisher>::iterator it;
  //for each joint in the JointState
  for(unsigned int i = 0; i < msg->name.size(); ++i)
  {
    std::string jointname = msg->name[i];
    // if the joint is actually controlled
    it=controller_maxforcepub_map.find(jointname);
		if(it!=controller_maxforcepub_map.end())
		{
			
			// CUTOUT REGION if above cutout force limit, increment a counter
			if(fabs(msg->effort[i]) > CUTOUT_FORCE_LIMIT)
			{
				force_cutout_counter[jointname]++;
				// if already too long above cutout force limit
				if (force_cutout_counter[jointname]>CUTOUT_FORCE_COUNT)
				{
					if(!forcecutout[jointname])
					{
						// reduce the force to low levels and continue to next joint
						ROS_ERROR("%s CUTOUT count reached, reducing the force",jointname.c_str());
						std_msgs::Float64 msg;
						msg.data=0.2;
						controller_maxforcepub_map[jointname].publish(msg);
						forcecutout[jointname]=true;
						continue;
					}
				}
				else
				{
					// if not too long increment and continue to next joint
					ROS_DEBUG("%s CUTOUT counter %d",jointname.c_str(),force_cutout_counter[jointname]);
					continue;
				}
			}
			else // if below cutout limit reset the counter
			{
				if(forcecutout[jointname])
				{
					force_cutout_counter[jointname]=0;
					ROS_DEBUG("%s OUT of CUTOUT region, counter zeroed",jointname.c_str());
				}
			
				// MONITORING REGION if in monitoring force region
				double curAbsError = fabs(msg->position[i] - previous_position[jointname]);
				previous_position[jointname]=msg->position[i];
				if(fabs(msg->effort[i]) > MONITOR_FORCE_LIMIT)
				{	
					// if position error is decreasing significantly in this region between each data reception
					
					if( (prevAbsError[jointname] - curAbsError) < LEAST_ACCEPTED_ERR_REDUCTION ) // reducing too slowly, we might be in contact
					{
						// increment the counter because we want to be sure we are in contact long enough					
						force_monitor_counter[jointname]++;
						// if already too long
						if(force_monitor_counter[jointname]>MONITOR_FORCE_COUNT)
						{
							if(!forcelimited[jointname])
							{
								// reduce the force to low levels and continue to next joint
								ROS_WARN("%s MONITORED TOO LOW CHANGES IN POSITION, reducing the force",jointname.c_str());
								std_msgs::Float64 msg;
								msg.data=0.5;
								controller_maxforcepub_map[jointname].publish(msg);
								forcelimited[jointname]=true;
								continue;
							}
						}
						else
						{
							ROS_DEBUG("%s MONITOR counter %d",jointname.c_str(),force_monitor_counter[jointname]);
						}
					}
					else // reducing rapidly enough, we are not in contact, reset counter
					{
						force_monitor_counter[jointname]=0;
						ROS_DEBUG("%s MONITOR counter zeroed",jointname.c_str());
						if(forcelimited[jointname])
						{
							force_monitor_counter[jointname]=0;
							std_msgs::Float64 msg;
							msg.data=0.7;
							controller_maxforcepub_map[jointname].publish(msg);
							ROS_WARN("%s Reducing correctly, no force limit anymore",jointname.c_str());
							forcelimited[jointname]=false;
						}						
					}
					
				}
				else
				{
					if(forcelimited[jointname] || forcecutout[jointname])
					{
						force_monitor_counter[jointname]=0;
						std_msgs::Float64 msg;
						msg.data=0.7;
						controller_maxforcepub_map[jointname].publish(msg);
						ROS_WARN("%s OUT OF MONITORING, no force or cutout limits anymore",jointname.c_str());
						forcelimited[jointname]=false;
						forcecutout[jointname]=false;
					}
				}
				prevAbsError[jointname]=curAbsError;
			}
		} 
  }
}


// initialize services to access setGains of each existing controller
void AgniGraspForceLimiter::initializeServices()
{
		if (!ros::service::waitForService
		 (nh.getNamespace() + "/pr2_controller_manager/list_controllers", ros::Duration(3.0))) {
      ROS_ERROR("couldn't find Shadow Hand controllers");
		return;
	}
		
	ros::ServiceClient ctrlListClient = nh.serviceClient<pr2_mechanism_msgs::ListControllers>
		("pr2_controller_manager/list_controllers");


	// get the controller list
	pr2_mechanism_msgs::ListControllers ctrlList;
	ctrlListClient.call(ctrlList);

	for (size_t i=0; i<ctrlList.response.controllers.size(); ++i) {
		const std::string& ctrlName = ctrlList.response.controllers[i];

		std::string jointName;
		std::string ctrlType;
		// check ctrl type
		if (nh.getParam(ctrlName+"/type", ctrlType)) {
			if(ctrlType.find(CTRLTYPE) != std::string::npos) { //correct type
				//  get controlled joint name
				if (nh.getParam(ctrlName+"/joint", jointName)) {
	
					size_t posJ0 = 0;
					if ((posJ0 = jointName.find("J0")) != std::string::npos) {
						jointName.replace (posJ0, 2, "J2");
					}
					controller_maxforcepub_map[jointName] = (nh.advertise<std_msgs::Float64>
													(ctrlName+"/max_force_factor",1)) ;
					previous_position[jointName] = 0.0;
					prevAbsError[jointName] = 90.0;
					forcelimited[jointName] = false;
					forcecutout[jointName] = false;
				
					ROS_INFO("controller %s controls joint %s\n",
								 ctrlName.c_str(), jointName.c_str());
				}
			} else { // wrong type 
				// For safety, check if the controller with wrong type is running
				if (ctrlList.response.state[i]=="running") {
					ROS_WARN ("controller %s is RUNNING but with wrong type %s",
								 ctrlName.c_str(), ctrlType.c_str());
					//TODO: stop the controllers
				}
			}
		} else {
			ROS_WARN ("Could not find the type of controller %s!",
						 ctrlName.c_str());
			throw std::runtime_error("could not get the type of controller : " 
											 + ctrlName);
		}
	}
}
  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "agni_grasp_force_limiter");

  AgniGraspForceLimiter hand_exec;
  ros::spin();

  return 0;
}
