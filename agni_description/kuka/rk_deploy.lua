require "rttlib"
require "rttros"

tc=rtt.getTC()
d=tc:getPeer("Deployer")

d:import("rtt_rosnode")
-- Start of user code imports
d:import("lwr_fri")
d:import("lwr_impedance_controller")
d:import("oro_joint_state_publisher")
-- End of user code

ros=false
prio=0
schedpol=rtt.globals.ORO_SCHED_OTHER

local opttab=utils.proc_args(arg)
local cp=rtt.Variable("ConnPolicy")

function conn2ros(depl, port, topic)
   depl:stream(port,rtt.provides("ros"):topic(topic))
end

d:loadComponent("LWRDiag", "FRIDiagnostics")
d:setActivity("LWRDiag", 0.02, 5, rtt.globals.ORO_SCHED_RT)
LWRDiag = d:getPeer("LWRDiag")
LWRDiag:configure()


d:loadComponent("FRI", "FRIComponent")
d:setActivity("FRI", 0.0, 5, rtt.globals.ORO_SCHED_RT)
FRI = d:getPeer("FRI")
FRI:getProperty("fri_port"):set(49940)
FRI:configure()


d:loadComponent("JntPub", "JointStatePublisher")
d:setActivity("JntPub", 0.01, 2, rtt.globals.ORO_SCHED_RT)
JntPub = d:getPeer("JntPub")
sched_order=JntPub:getProperty("joint_names")
sched_order:get():resize(7)
sched_order[0]="right_arm_0_joint"
sched_order[1]="right_arm_1_joint"
sched_order[2]="right_arm_2_joint"
sched_order[3]="right_arm_3_joint"
sched_order[4]="right_arm_4_joint"
sched_order[5]="right_arm_5_joint"
sched_order[6]="right_arm_6_joint"
JntPub:configure()

--d:loadComponent("FTMsr", "FrameTransformer")
--d:setActivity("FTMsr", 0.01, 2, rtt.globals.ORO_SCHED_RT)
--FTMsr = d:getPeer("FTMsr")
--FTMsr:configure()

--d:loadComponent("FTCmd", "FrameTransformer")
--d:setActivity("FTCmd", 0.01, 2, rtt.globals.ORO_SCHED_RT)
--FTCmd = d:getPeer("FTCmd")
--FTCmd:configure()


d:stream("LWRDiag.Diagnostics",rtt.provides("ros"):topic("diagnostics"))

d:connect("FRI.RobotState", "LWRDiag.RobotState", rtt.Variable("ConnPolicy"))
d:connect("FRI.FRIState", "LWRDiag.FRIState", rtt.Variable("ConnPolicy"))
d:connect("FRI.JointPosition", "JntPub.msrJntPos", rtt.Variable("ConnPolicy"))
d:stream("FRI.KRL_CMD",rtt.provides("ros"):topic("lwr_arm_controller/fri_set_mode"))
d:stream("JntPub.joints_state",rtt.provides("ros"):topic("joint_states"))

--d:stream("FTMsr.FrameOutput",rtt.provides("ros"):topic("/msr/pose_tool_msr"))
--d:stream("FTCmd.FrameOutput",rtt.provides("ros"):topic("/msr/pose_tool_cmd"))

-- Start of user code usercode
FRI:start()
LWRDiag:start()
JntPub:start()

--FTMsr:start()
--FTCmd:start()
--d:waitForSignal()
-- End of user code

