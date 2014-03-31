#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sr_robot_msgs/UBI0All.h>
#include <sr_robot_msgs/MidProxDataAll.h>
#include <sr_robot_msgs/AuxSpiData.h>
#include <std_msgs/Float64MultiArray.h> 
#include <boost/functional/hash.hpp>

#include <string>

#include <boost/thread.hpp>

//messages
#include <std_msgs/Float64.h>

#define COUNTDOWN_MAX   5
#define FF      0
#define MF      1
#define RF      2
#define LF      3
#define TH      4

#define	NB_SENSOR	5

//ros subscriber (will be instantiated later on)
ros::Subscriber sub[5];
ros::Publisher marker_pub;
std_msgs::Float64::_data_type data[5];

boost::shared_mutex update_mutex; // multiple reads / one write mutex

int colors[8][3]={{0,0,0},{1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1},{1,1,1}};

std::vector<int>          vShadowTipID2MyTipID(5);

int  flip = 1;

std::string tf_prefix = "";

struct tactile_marker{
	std::string name;
	std::string fingerPrefix;
	std::string meshpath;
	double val;
};

// vector of Tactile vector data
struct distal_tactile{
	std::vector<tactile_marker> markers; //12
};

struct midprox_tactile{
	std::vector<tactile_marker> midmarkers; //4
	std::vector<tactile_marker> proxmarkers; //4
	
};

struct palm_tactile{
	std::vector<tactile_marker> markers; //4
}; 

struct aux_tactile {
	std::vector<tactile_marker> markers; //16
}; 


std::vector<distal_tactile> vTactileData(5);
std::vector<midprox_tactile> vMidProxTactileData(5);
std::vector<aux_tactile> vAuxSpiTactileData(1);
std::vector<palm_tactile> vPalmExtrasData(1);


// Set our initial shape types
uint32_t default_shape = visualization_msgs::Marker::ARROW;
uint32_t mesh_shape = visualization_msgs::Marker::MESH_RESOURCE;

std::string sFinger[5]={"th","ff","mf","rf","lf"};
std::string ns="";

void publish_marker(unsigned int id, std::string framesuffix,  float pressure, std::string mesh_resource="")
{

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = framesuffix;
	marker.header.stamp = ros::Time(0);//::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "tactile";
	marker.id = id;

	geometry_msgs::Point mypoint;
	// Set the marker type.  
	
	if (mesh_resource=="")
	{
		marker.type = default_shape;
		// Set the pose of the marker. 
		// Set start point
		float phalanx_thickness=0.007;
		float phalanx_length=0.01;
		mypoint.x= 0;
		mypoint.y= phalanx_thickness;
		mypoint.z= phalanx_length;
		marker.points.push_back(mypoint);

		// Set end point
		mypoint.x= 0;
		mypoint.y= (phalanx_thickness+pressure/20.0);
		mypoint.z= phalanx_length;
		marker.points.push_back(mypoint);

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.0025;
		marker.scale.y = 0.004;
		marker.scale.z = 0;
	}
	else
	{
		marker.type = mesh_shape; 
		marker.mesh_resource = mesh_resource;
		// Set the scale of the marker
		if ( mesh_resource.find("palm")!=std::string::npos || mesh_resource.find("meta")!=std::string::npos)
			marker.scale.x = flip*0.001;
		else
			marker.scale.x = 0.001;
		marker.scale.y = 0.001;
		marker.scale.z = 0.001;
	}

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;
	// Set the color -- be sure to set alpha to something non-zero!
	//if (pressure < 0.01)
	//	marker.color.a = 0.0;
	//else {
		//if (pressure >=0.01 && pressure < 0.1)
		//	marker.color.a = pressure*10;
		//else
			marker.color.a = 1.0;
	//	}
		
	marker.color.r = (pressure)>0.3?1.5*(pressure-0.3):0.0;
	marker.color.g = pressure>0.3?(1.50f-(pressure*1/0.66)):3*pressure;
	marker.color.b = 0.0f;
	
	marker.lifetime = ros::Duration();

	// Publish the marker
	marker_pub.publish(marker);
}

void publish_all_markers()
{
	double val = 0.0;
	boost::hash<std::string> string_hash;
	for (unsigned int iFinger=1;iFinger<NB_SENSOR;++iFinger)
	{
		// tips
		for (size_t i=0;i<vTactileData[iFinger].markers.size();++i)
		{
			val = vTactileData[iFinger].markers[i].val;
			//if(val > 0.01)
			//{
				std::size_t h = abs(string_hash(tf_prefix+vTactileData[iFinger].markers[i].fingerPrefix+vTactileData[iFinger].markers[i].name));
				publish_marker(h,tf_prefix+vTactileData[iFinger].markers[i].fingerPrefix+"distal", val,vTactileData[iFinger].markers[i].meshpath); 
			//}
		}
		// middle 
		for (size_t i=0;i<vMidProxTactileData[iFinger].midmarkers.size();++i)
		{
			val = vMidProxTactileData[iFinger].midmarkers[i].val;
			//if(val > 0.01)
			//{
				std::size_t h = abs(string_hash(tf_prefix+vMidProxTactileData[iFinger].midmarkers[i].fingerPrefix+vMidProxTactileData[iFinger].midmarkers[i].name));
				publish_marker(h,tf_prefix+vMidProxTactileData[iFinger].midmarkers[i].fingerPrefix + "middle", val,vMidProxTactileData[iFinger].midmarkers[i].meshpath); 
			//}
		}
		// proximal
		for (size_t i=0;i<vMidProxTactileData[iFinger].proxmarkers.size();++i)
		{
			val = vMidProxTactileData[iFinger].proxmarkers[i].val;
			//if(val > 0.01)
			//{
				std::size_t h = abs(string_hash(tf_prefix+vMidProxTactileData[iFinger].proxmarkers[i].fingerPrefix+vMidProxTactileData[iFinger].proxmarkers[i].name));
				publish_marker(h,tf_prefix+vMidProxTactileData[iFinger].proxmarkers[i].fingerPrefix + "proximal", val,vMidProxTactileData[iFinger].proxmarkers[i].meshpath); 
			//}
		}
		
		ROS_DEBUG ("finger %d",iFinger);
		ROS_DEBUG ("tip tactile 8 val %lf",vTactileData[iFinger].markers[8].val);
		ROS_DEBUG ("mid tactile 1 val %lf",vMidProxTactileData[iFinger].midmarkers[1].val);
		ROS_DEBUG ("prox tactile 1 val %lf",vMidProxTactileData[iFinger].proxmarkers[1].val);
	}
	
	// aux palm
	for (size_t i=0;i<vAuxSpiTactileData[0].markers.size();++i)
	{
		val = vAuxSpiTactileData[0].markers[i].val;
		//if(val > 0.01)
		//{
			std::size_t h = string_hash(tf_prefix+vAuxSpiTactileData[0].markers[i].name);
			publish_marker(h,tf_prefix+"palm", val,vAuxSpiTactileData[0].markers[i].meshpath); 
		//}
	}
	//palm extras
	for (size_t i=0;i<vPalmExtrasData[0].markers.size();++i)
	{
		val = vPalmExtrasData[0].markers[i].val;
		//if(val > 0.01)
		//{
			std::size_t h = string_hash(tf_prefix+vPalmExtrasData[0].markers[i].name);
			if (i==3)
				publish_marker(h,tf_prefix+"palm", val,vPalmExtrasData[0].markers[i].meshpath); 
			else
				publish_marker(h,tf_prefix+"lfmetacarpal", val,vPalmExtrasData[0].markers[i].meshpath); 
		//}
	}
	
	
	ROS_DEBUG ("palm");
	ROS_DEBUG ("aux tactile 5 val %lf",vAuxSpiTactileData[0].markers[5].val);
	ROS_DEBUG ("palm extras tactile 3 val %lf",vPalmExtrasData[0].markers[3].val);
}


void init_markers()
{
	tactile_marker marker;
	for (unsigned int iFinger=1;iFinger<NB_SENSOR;++iFinger)
	{
		marker.fingerPrefix=sFinger[iFinger];
		marker.meshpath = "";
		std::stringstream ssname,sspath;
		for (unsigned int iTaxel=0; iTaxel < 12 ; ++iTaxel)
		{
			ssname << "adc" << iTaxel;
			marker.name = ssname.str();
			sspath << "package://sr_description/hand/model/ubi_tactiles/tax_tip_" <<iTaxel <<".stl";
			marker.meshpath = sspath.str();
			vTactileData[iFinger].markers.push_back(marker);
			ssname.str(std::string());
			sspath.str(std::string());
		}
		
		marker.name = "inactive";
		marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_mid_l.stl";
		vMidProxTactileData[iFinger].midmarkers.resize(4,marker);
		marker.name = "mid_l";
		vMidProxTactileData[iFinger].midmarkers[0] = marker;
		marker.name = "mid_r";
		marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_mid_r.stl";
		vMidProxTactileData[iFinger].midmarkers[1] = marker;
		marker.name = "inactive";
		marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_prox_r.stl";
		vMidProxTactileData[iFinger].proxmarkers.resize(4,marker);
		marker.name = "prox_r";
		vMidProxTactileData[iFinger].proxmarkers[0] = marker;
		marker.name = "prox_m";
		marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_prox_m.stl";
		vMidProxTactileData[iFinger].proxmarkers[1] = marker;
		marker.name = "prox_l";
		marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_prox_l.stl";
		vMidProxTactileData[iFinger].proxmarkers[2] = marker;
		
	}
	marker.fingerPrefix="";
	marker.name = "inactive";
	vAuxSpiTactileData[0].markers.resize(16);
	marker.name = "palm_up_r";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_palm_up_r.stl";
	vAuxSpiTactileData[0].markers[0] = marker;
	marker.name = "palm_up_mid";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_palm_up_mid.stl";
	vAuxSpiTactileData[0].markers[1] = marker;
	marker.name = "palm_up_l";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_palm_up_l.stl"; 
	vAuxSpiTactileData[0].markers[2] = marker;
	marker.name = "palm_center_r";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_palm_center_r.stl";
	vAuxSpiTactileData[0].markers[3] = marker;
	marker.name = "palm_center_l";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_palm_center_l.stl";
	vAuxSpiTactileData[0].markers[4] = marker;
	marker.name = "palm_center_mid";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_palm_center_mid.stl";
	vAuxSpiTactileData[0].markers[5] = marker;
	marker.name = "palm_down_r";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_palm_down_r.stl";
	vAuxSpiTactileData[0].markers[6] = marker;
	marker.name = "palm_down_l";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_palm_down_l.stl";
	vAuxSpiTactileData[0].markers[7] = marker;
	
	marker.name = "inactive";
	vPalmExtrasData[0].markers.resize(4);
	marker.name = "metac_upper";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_metac_upper.stl";
	vPalmExtrasData[0].markers[0] = marker;
	marker.name = "metac_lower";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_metac_lower.stl";
	vPalmExtrasData[0].markers[1] = marker;
	marker.name = "metac_side";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_metac_side.stl";
	vPalmExtrasData[0].markers[2] = marker;
	marker.name = "palm_side";
	marker.meshpath = "package://sr_description/hand/model/ubi_tactiles/tax_palm_side.stl"; 
	vPalmExtrasData[0].markers[3] = marker;
}
/*
// Generic callback 
void callback(const kcl_msgs::KCL_ContactStateStampedConstPtr &msg, std::string tipname)
{
    //ROS_INFO("Got a ContactState message on topic %s", tipname.c_str());
    const kcl_msgs::KCL_ContactStateStamped v = *msg;
    boost::hash<std::string> string_hash;
    std::size_t h = string_hash(tipname);
    if(fabs(v.Fnormal)>5.0)
        publish_marker_special(h,tipname, v,1); //Red
    else if(fabs(v.Fnormal)>2.0 && fabs(v.Fnormal) <= 5.0)
        publish_marker_special(h,tipname, v,4); //Orange
    else
        publish_marker_special(h,tipname, v,2); //Green
}
*/

// receive Tactile data from UBI Fingertips   
void 
recvTipTactile(const sr_robot_msgs::UBI0AllConstPtr& msg)
{
	// get access at all
	boost::upgrade_lock<boost::shared_mutex> lock(update_mutex);
	// upgrade access to unique write access
	boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);

	// for each sensor
	for(size_t i = 0; i < msg->tactiles.size(); ++i) {
		int MyTipID = vShadowTipID2MyTipID[i];
		if(MyTipID!=0) //thumb has not data yet
		{
			assert(vTactileData[MyTipID].markers.size() == msg->tactiles[i].distal.size() );
		
			for(size_t j = 0; j < msg->tactiles[i].distal.size(); ++j) {
				float val = static_cast<float>(msg->tactiles[i].distal[j]);
				if (val >0 && val <= 1023)
					vTactileData[MyTipID].markers[j].val = (1024.0-val)/1024.0;
				else
					vTactileData[MyTipID].markers[j].val = 0.0;
			}
		}
	}
}

// receive Tactile data from UBI mid prox tactile sensors   
void 
recvMidProxTactile(const sr_robot_msgs::MidProxDataAllConstPtr& msg)
{
	// get access at all
	boost::upgrade_lock<boost::shared_mutex> lock(update_mutex);
	// upgrade access to unique write access
	boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);

	// for each sensor
	for(size_t i = 0; i < msg->sensors.size(); ++i) {
		int MyTipID = vShadowTipID2MyTipID[i];
		if(MyTipID!=0) //thumb has not data yet
		{
			assert( vMidProxTactileData[MyTipID].proxmarkers.size() == msg->sensors[i].proximal.size());
			assert( vMidProxTactileData[MyTipID].midmarkers.size() == msg->sensors[i].middle.size());
			
			for(size_t j = 0; j < msg->sensors[i].middle.size(); ++j) {

				vMidProxTactileData[MyTipID].midmarkers[j].val = (static_cast<float>(msg->sensors[i].middle[j]))/2048.0;
			}
			for(size_t j = 0; j < msg->sensors[i].proximal.size(); ++j) {
				vMidProxTactileData[MyTipID].proxmarkers[j].val =  (static_cast<float>(msg->sensors[i].proximal[j]))/2048.0;
			}
		}
	}
}

// receive Tactile data from palm aux spi tactile sensors   
void 
recvAuxSpiTactile(const sr_robot_msgs::AuxSpiDataConstPtr& msg)
{
	// get access at all
	boost::upgrade_lock<boost::shared_mutex> lock(update_mutex);
	// upgrade access to unique write access
	boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);

	assert (vAuxSpiTactileData[0].markers.size() == msg->sensors.size() );
	// for each sensor	
	for(size_t j = 0; j < msg->sensors.size(); ++j) {
		vAuxSpiTactileData[0].markers[j].val = (static_cast<float>(msg->sensors[j]))/1024.0;
	}
}

// receive Tactile data from palm aux analog sensors   
void 
recvPalmExtras(const std_msgs::Float64MultiArrayConstPtr& msg)
{
	// get access at all
	boost::upgrade_lock<boost::shared_mutex> lock(update_mutex);
	// upgrade access to unique write access
	boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(lock);
	size_t dataoffset=0, datasize=0;
	bool analog_data_found=false;
	for(size_t i = 0; i < msg->layout.dim.size(); ++i) {
		// check for the analog data
		if (msg->layout.dim[i].label.find("analog_inputs")!=std::string::npos){
			datasize= msg->layout.dim[i].size;
			analog_data_found=true;
			break;
		}
		// accumulate previous seen size whose label does not match to get the overalloffset
		dataoffset+=msg->layout.dim[i].size;
	}
	if (analog_data_found) {
		assert (vPalmExtrasData[0].markers.size() == datasize );
		// for each sensor	
		for(size_t j = 0; j < datasize; ++j) {
			vPalmExtrasData[0].markers[j].val = (static_cast<float>(msg->data[dataoffset+j]))/1024.0;
		}
	}
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "agni_tactile_viz");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ns = ros::this_node::getNamespace();
	
	if (ns != "" )
	{
		// remove first slash if any
		if (ns[0] == '/')
			ns=ns.substr(1,ns.size()-1);
			
		ns=ns+"/";
	}
	
	if (ns=="/lh/")
	{
		flip = -1;
		ROS_INFO("Flipping meshes");
	}
	if (nh_priv.hasParam("tf_prefix"))
		nh_priv.getParam("tf_prefix", tf_prefix);
	else 
		ROS_WARN("Did not find tf_prefix");
		
	
	ROS_INFO_STREAM("using " << ns << " namespace" << " using tf_prefix " << tf_prefix);
		
	ros::Rate r(50);
	marker_pub = nh.advertise<visualization_msgs::Marker>("agni_tactile_markers", 1);
	
	// create a map for the tactile order
	vShadowTipID2MyTipID[0]=1;
	vShadowTipID2MyTipID[1]=2;
	vShadowTipID2MyTipID[2]=3;
	vShadowTipID2MyTipID[3]=4;
	vShadowTipID2MyTipID[4]=0;

	init_markers();
	
	// create subscribers
	ros::Subscriber	tacTipSub,tacMidProxSub,tacAuxSpiSub,tacPalmExtrasSub;
	tacTipSub=nh.subscribe("tactile", 1, // buffer size
								  &recvTipTactile); 
	tacMidProxSub=nh.subscribe("tactile_mid_prox", 1, // buffer size
								  &recvMidProxTactile);
	tacAuxSpiSub=nh.subscribe("tactile_aux_spi", 1, // buffer size
								  &recvAuxSpiTactile);
	tacPalmExtrasSub=nh.subscribe("palm_extras", 1, // buffer size
								  &recvPalmExtras);
  
  while (ros::ok())
  {
		publish_all_markers();
    r.sleep();
    ros::spinOnce();
  }
}
