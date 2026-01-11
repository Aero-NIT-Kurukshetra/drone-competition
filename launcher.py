import sys

# workers
from workers.mission_manager.main import runner as mm_runner
from workers.lidar_processing.main import runner as lp_runner
from workers.path_planning.main import runner as pp_runner
# from workers.camera_processing.main import runner as cp_runner

# tests
from test_scripts.lidar_test.streamer import runner as lidar_streamer   # streams lidar data over mavlink
from test_scripts.lidar_test.receiver import runner as lidar_receiver   # plots data received from mavlink
from test_scripts.path_planning_test.mock_start_mission import runner as kml_streamer  # streams kml data to path planner


links = {
	# workers
	"mission_manager": mm_runner,
	"lidar_processing": lp_runner,
	"path_planner": pp_runner,
	# "camera_processing": cp_runner,

	# tests
	"lidar_streamer": lidar_streamer,
	"lidar_receiver": lidar_receiver,
	"kml_streamer": kml_streamer
}

if __name__ == "__main__":
	if len(sys.argv) != 2:
		print(f"Usage: python launcher.py [script_name]\nAvailable scripts: {', '.join(links.keys())}")
		sys.exit(1)

	script = sys.argv[1]
	
	if script in links:
		links[script]()
	else:
		print(f"Unknown script: {script}")
		sys.exit(1)