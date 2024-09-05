# === Start TFs docker === #
.PHONY: tf
tf:
	@echo "Starting TFs ..."
	@sudo bash -c "ros2 launch robot robot.launch.py"

# === Gazebo docker === #
.PHONY: sim
sim:
	@echo "Starting Simulations ..."
	@sudo bash -c "ros2 launch robot sim.launch.py  world:=/ws_navtech/src/robot/worlds/simulation.world"

# === Gazebo docker === #
.PHONY: sim2
sim2:
	@echo "Starting Simulations ..."
	@sudo bash -c "ros2 launch robot sim.launch.py"

# === Gazebo docker === #
.PHONY: sim3
sim3:
	@echo "Starting Simulations ..."
	@sudo bash -c "ros2 launch robot hospital.launch.py"

# === Teleop docker === #
.PHONY: teleop
teleop:
	@echo "Starting Teleop ..."
	@sudo bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard"

# === SLAM ToolBox docker === #
.PHONY: slam
slam:
	@echo "Starting SLAM ToolBox ..."
	@sudo bash -c "ros2 launch robot slam.launch.py"

# === AMCL docker === #
.PHONY: amcl
amcl:
	@echo "Starting AMCL ..."
	@sudo bash -c "ros2 launch robot localization.launch.py use_sim_time:=true map_yaml_file:=/ws_navtech/src/navtech/robot/maps/map.yaml"

# === Nav docker === #
.PHONY: nav
nav:
	@echo "Starting Nav2 ..."
	@sudo bash -c "ros2 launch robot navigation.launch.py use_sim_time:=true"

# === Save Map docker === #
.PHONY: save
save:
	@echo "Saving Map ..."
	@sudo bash -c "ros2 run nav2_map_server map_saver_cli -f /ws_navtech/src/navtech/robot/config/map"

# === Run === #
.PHONY: run
run:
	@echo "Starting ..."
	@sudo bash -c "ros2 launch robot run.launch.py"
