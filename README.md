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
