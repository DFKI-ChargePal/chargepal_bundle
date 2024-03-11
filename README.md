# chargepal_bundle
`chargepal_bundle` package is the root package that loads config parameters and starts all necessary nodes for the robot.  

## Dependencies
- nlohmann-json3-dev : `sudo apt-get -y install nlohmann-json3-dev`

## Installation
- Create a `catkin_ws` folder and a `src` folder inside it.
- Clone the following packages `chargepal_bundle`,`chargepal_behaviour_tree`,`chargepal_client`,`chargepal_actions`,`chargepal_services`inside a `catkin_ws/src`.
- Add submodules by running `git submodule init` and `git submodule update`. 
- Inside `catkin_ws/src` run `ln -s chargepal_behaviour_tree/BehaviorTree.CPP`.This creates a softlink to the BehaviorTree.CPP that is present inside the `chargepal_behaviour_tree` package, to outside the package.
- Run `catkin_make` inside `/catkin_ws`.
 
## Checklist before starting the nodes
- [ ] Make sure the local database is created. Set up the envionment representation in the local database (server). 
- [ ] Make sure missions are added inside MiR platform. <details><summary>Click to see how missions are added</summary></details>
- [ ] Configure the `cfg/config.yaml` file with necessary values.
- [ ] Check if the `.py` and `.pyi` files are created inside `/chargepal_client/src/chargepal_client`. If not, inside `/chargepal_client/src/chargepal_client` run `python -m grpc_tools.protoc -I. --python_out=. --pyi_out=. --grpc_python_out=. communication.proto ` to create them.

## Execution

**Step 1:** Set the configuration file

| Parameter | Example |Notes |
| ------ | ------ |------ |
|  server_address      |   "localhost:50059"     |Make sure to set the ip/hostname and port number of [chargepal_local_server](https://git.ni.dfki.de/chargepal/system-integration/server-packages/chargepal_local_server/-/blame/main/src/chargepal_local_server/server.py?ref_type=heads#L118)|
|     mir_address   | "192.168.12.20"       | MiR adress can be found in the [wiki](https://git.ni.dfki.de/chargepal/chargepal_wiki/-/wikis/Documentation/onboard-devices-and-network)|
|     robot_name   | 'ChargePal1'       |The name of the robot. Make sure to follow the same naming convention|
|     robot_id   | 'ER-FLEX-00040'     |The robot id mentioned on each robot|
|     sim_flag   | True      |Set this value to `True` to simulate robot actions as sleep intervals.Else, set it to `False` if you want to see the robot perform the action|
|     recovery_enabled   | False       |Set this to `True` to have recovery actions enabled for any failed action. Else, set it to `False` to pause and ask for manual recovery|
|     server_timeout   | 600       | Value in seconds which denote the time interval after which a technician is to be called when connection to server has failed|


**Step 3:** `catkin_make` and `source devel/setup.bash` the workspace
**Step 2:** Run `roslaunch chargepal_bundle startup.launch`. Wait for a few seconds for it to finish starting all the launch files. Then run `rosrun chargepal_behaviour_tree behaviour_tree_cpp`.
