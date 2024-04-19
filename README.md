# chargepal_bundle
`chargepal_bundle` package is the root package that loads config parameters and starts all necessary nodes for the robot.  

## Dependencies
- nlohmann-json3-dev : `sudo apt-get -y install nlohmann-json3-dev`

| package | branch |
| ------ | ------ |
|   [chargepal_local_server](https://git.ni.dfki.de/chargepal/system-integration/server-packages/chargepal_local_server/-/tree/hannover_gui?ref_type=heads)      |   hannover_gui     |
|  [chargepal_monitor_gui](https://git.ni.dfki.de/chargepal/system-integration/server-packages/chargepal_monitor_gui/-/tree/hannover_gui?ref_type=heads)      |   hannover_gui     |
|     [chargepal_actions](https://git.ni.dfki.de/chargepal/system-integration/robot-packages/chargepal_actions)    |   hannover_gui     |
|    [chargepal_behaviour_tree](https://git.ni.dfki.de/chargepal/system-integration/robot-packages/chargepal_behaviour_tree)    |    hannover_gui    |
|    [chargepal_bundle](https://git.ni.dfki.de/chargepal/system-integration/robot-packages/chargepal_bundle)    |    hannover_gui    |
|    [chargepal_services](https://git.ni.dfki.de/chargepal/system-integration/robot-packages/chargepal_services)    |    main    |
| [chargepal_map](https://git.ni.dfki.de/chargepal/manipulation/chargepal_map/-/tree/feat/start_state?ref_type=heads) | feat/start_state|

- Ensure the `chargepal_local_server` is placed outside the `chargepal_ws`

## Installation
- Create a `catkin_ws` folder and a `src` folder inside it.
- Clone the following packages `chargepal_bundle`,`chargepal_behaviour_tree`,`chargepal_client`,`chargepal_actions`,`chargepal_services`inside a `catkin_ws/src`.
- Add submodules by running `git submodule init` and `git submodule update`. 
- Inside `catkin_ws/src` run `ln -s chargepal_behaviour_tree/BehaviorTree.CPP`.This creates a softlink to the BehaviorTree.CPP that is present inside the `chargepal_behaviour_tree` package, to outside the package.
- Run `catkin_make` inside `/catkin_ws`.
 
## Checklist before starting the nodes
- [ ] Make sure the local database is created. Set up the envionment representation in the local database (server). 
- [ ] Make sure missions are added inside MiR platform.
- [ ] Make sure `gui.yaml` (chargepal_monitor_gui) is set with the required values. 
- [ ] Make sure `config.yaml` (chargepal_bundle) is set with the required values. 
- [ ] `./create_ldb.py` whenever the enviroinment is reset to the base position ( robot and carts in their respective bases).
- [ ] Check if the `.py` and `.pyi` files are created inside `/chargepal_client/src/chargepal_client`. If not, inside `/chargepal_client/src/chargepal_client` run `python -m grpc_tools.protoc -I. --python_out=. --pyi_out=. --grpc_python_out=. communication.proto ` to create them.

## Execution
1. **Environment setup**
- Generate a map inside the mir platform .
- Create charger as a markers on the created map.
- Create positions (E.g. ADS_1, ADS_1_pick , BWS_1, BWS_1_pick) on the created map.
- Make sure every mission inside mission group "Chargepal" is associated with the markers and position marked in the set map. Remember to click Save! 
- Manually execute every mission, to check if the positions are reachable. 

2. **Setup the manipulation**
- View [ChargePal Manipulation Setup](https://git.ni.dfki.de/-/snippets/9) for further detials to setup the manipulation

3. **On our Orin** 
- ssh `chargepal@192.168.12.50` (only for Orin)
- Make sure `gui.yaml` (chargepal_monitor_gui) is set with the required values. 
- Make sure `config.yaml` (chargepal_bundle) is set with the required values. 
- `./create_ldb.py` whenever the enviroinment is reset to the base position ( robot and carts in their respective bases).

- Sever node (tmux): 
    - `./server.py` to run the server 
- Robot nodes (tmux):
    - `roslaunch chargepal_bundle startup.launch` 
    - `roslaunch chargepal_map action_server.launch` 
    - `rosrun chargepal_behaviour_tree chargepal_behaviour_tree`

4. **GUI**
- Open `localhost:8080` on a browser to view the gui. 
- Under Demo Control -> set a demo loop value and press START.
- **NOTES**: 
    - Press STOP to stop after the ongoing job.
    - After STOP is pressed and **only after** the ongoing job is completed:
        - Press RESUME to resume with the ongoing demo loop
        - Press START to start the demo again with the value in the demo loop.
## Troubleshooting
- When `recovery_enabled` is `False`
    - The robot stops upon a failure and asks for a manual rectification.
- **DO NOT** stop the `chargepal_local_server` node. If done, please put the robot and cart in the base position and restart the entire execution procedure.


| Failure action | Problem resolution |
| ------ | ------ |
|  arrive_at_station arrive_at_home      |  **(1)** Make sure the target position has no obstacles. **(2)** Try creating some space around the target position. **(3)** Release emergency stop button if pressed. **(4)** Press RESUME in the gui to continue performing the action.    |
|   pickup_cart    |  **(1)** Make sure the robot is in front of the cart. **(2)** Adjust the cart position slightly if necessary **(3)** Ensure the robot is restored to its previous position prior to the `pickup_cart`. **(4)** Press RESUME in the gui to continue performing the action.  |
|   place_cart    |  **(1)** Make sure the space behaind the robot is free. **(2)** Adjust the cart position slightly if necessary **(3)** Ensure the robot is restored to its previous position prior to the `place_cart`. **(4)** Press RESUME in the gui to continue performing the action.     |
|plugin_ADS plugout_ADS|**(1)** Make sure the emergency switch is released and the robot arm is started via the teach pendant **(2)** Under **Features** -> **Recover Arm** in the gui, press **SET ARM FREE** to set the arm free. Move the arm by hand to a suitable position. Then press **STOP FREE ARM**. **(3)** Ensure the robot is restored to its previous position prior to the failed action (`plugin_ADS` or `plugout_ADS`). **(4)** Under **Features** -> **Recover Arm** in the gui, press **MOVE HOME** to move the arm to home position. **(4)** Press RESUME in the gui to continue performing the action. 
