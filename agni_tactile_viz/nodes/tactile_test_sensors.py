#!/usr/bin/env python

import roslib; roslib.load_manifest('agni_tactile_viz')
 
import rospy
from sr_robot_msgs.msg import UBI0All,UBI0, AuxSpiData, MidProxDataAll, MidProxData
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

class AgniTactileTestSensors():
  
    def __init__(self):
      
      
      
      pub1 = rospy.Publisher('tactile', UBI0All)
      pub2 = rospy.Publisher('palm_extras', Float64MultiArray)
      pub3 = rospy.Publisher('tactile_mid_prox', MidProxDataAll)
      pub4 = rospy.Publisher('tactile_aux_spi', AuxSpiData)
      
      ubiall = UBI0All()
      ubiall.tactiles=[]
      ubi= UBI0()
      ubi.distal = [200, 300, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 900]
      midproxall = MidProxDataAll()
      midproxall.sensors=[]
      midprox = MidProxData()
      midprox.middle = [1500, 2400, 0, 0]
      midprox.proximal = [100, 400, 800, 0]
            
      for iFinger in range(0,5):
        ubiall.tactiles.append(ubi)
        midproxall.sensors.append(midprox)
      
      palmextras = Float64MultiArray()
      dim = MultiArrayDimension()
      dim.label = "accelerometer"
      dim.size = 3
      palmextras.layout.dim.append(dim)
      dim = MultiArrayDimension()
      dim.label = "gyrometer"
      dim.size = 3
      palmextras.layout.dim.append(dim)
      dim = MultiArrayDimension()
      dim.label = "analog_inputs"
      dim.size = 4
      palmextras.layout.dim.append(dim)
      palmextras.data =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 250.0, 550.0,850.0, 1150.0]
      
      auxspi = AuxSpiData()
      auxspi.sensors = [100, 400, 700, 1000, 1300, 1700, 2000, 2300, 100, 200, 300, 400, 500, 600, 700, 800]


      rospy.init_node('agni_tactile_test_sensors')
      
      r = rospy.Rate(100) # 100hz
      
      while not rospy.is_shutdown():
        
        pub1.publish(ubiall)
        pub2.publish(palmextras)
        pub3.publish(midproxall)
        pub4.publish(auxspi)
        r.sleep()


# start the script
if __name__ == "__main__":
    agnitestsensor = AgniTactileTestSensors()
  
