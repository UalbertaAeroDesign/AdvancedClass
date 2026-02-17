# SETUP TRICOPTER TO RUN PRECISION LAND SCRIPT SIM IN GAZEBO

## STEP 1:
First, you need to make sure you have the appropriate Gazebo model folder "minihawk_vtol". IF you go to the github, youll see I did a monolith push of the entire gz_ws repository into our organization. Ours is also called gz_ws. You can clone this and make it your functioning gazebo direcotry, or you can clone it somewhere else and just grab the necessary files and folders. I recommend the first option, however since all my changes ive made span across numerous files. 

## STEP 2:
If your gazebo is the exact same as the one in our github, then you should have all the appropraite models and worlds to run the simulation. 


### Terminal 1 (GAZEBO SERVER):
navigate to your gazebo directory (gz_ws/src/ardupilot_gazebo) and run

    gz sim -s -v4 -r worlds/minihawk_runway.sdf

### Terminal 2 (GUI):
navigate to your gazebo directory (gz_ws) and run

    gz sim -g -v4

### TERMINAL 3 (ArduPilot SITL):
in your ardupilot directory  directory, run

    sim_vehicle.py -v ArduPlane -f JSON --out=127.0.0.1:14550 --out=127.0.0.1:14551

CRUCIAL: Load the parameters for the minihawk vtol plane. These are in ~/gz_ws/src/ardupilot_gazebo/config
in the ardupilot terminal, once drone has connected (verify in either QGC or MissionPlanner) run 

    param load ~/gz_ws/src/ardupilot_gazebo/config/minihawk_vtol.param

and then restart ardupilot SITL for changes to take effect.
Your exact path may be a bit different (though it probably shouldnt be). 
Note that this migth not work fully the first time. There may not even be warning messages saying it didnt work...
You know pretty quick if it didnt work by going into your SITL terminal and running:

     mode qstabilize

If you see the propellers on the tricoper move/point straight up, youre probably all good. If not, maybe run this step again. If that still doesnt work, go to your
parameter list in either QGC or mission planner and verify these parameters:

Q_ENABLE         1
Q_TILT_ENABLE    1
Q_TILT_FIX_ANGLE 0.0
Q_TILT_FIX_GAIN  0.0
Q_TILT_MASK      3
Q_TILT_MAX       55
Q_TILT_RATE_DN   40
Q_TILT_RATE_UP   40
Q_TILT_TYPE      2
Q_TILT_WING_FLAP 0.0
Q_TILT_YAW_ANGLE 14.0

if yours differ, set them to be consistent with the values above.

### TERMINAL 4:
This terminal runs the actual code, so i usually have this as a vscode terminal. Naviagte to /UalbertaAeroDesign/AdvancedClass/GazeboSimulation/tricoper
First, enable downcam streaming by running:

    gz topic -t /world/runway/model/minihawk_vtol/link/down_cam_link/sensor/down_cam/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
    
NOTE: Every time you restart the gazebo terminal, you need to rerun the above command.

then in the same terminal, run

    python tri_precision_land_full.py

observe gazebo gui window. Does the tricopter shoot up, hover, open camera, and eventually land? If so, youre set! You can now start to edit this tri_precision_land_full.py script to add elements like transition from fixed to vtol mode, april tag detection for precision loiter and land, and anything else 
youd like.



### MISC
Configure correct tricopter settings: read https://ardupilot.org/plane/docs/guide-tilt-rotor.html

