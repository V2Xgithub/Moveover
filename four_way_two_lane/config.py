# file paths for the SUMO simulation configuration and output results
sumo_cfg = r"sumocfg/four_way_two_lane.sumocfg"
output_path = 'results/four_way_two_lane.xml'

# SUMO parameters
step_length = "0.01"  # discrete step used by SUMO
simulation_steps = 3000000  # simulation length

# intersection layout parameters
negotiation_length = 10  # length of each negotiation zone
distance_to_conflict_zones = 104  # distance from the beginning of the negotiation zone to the first conflict zone
control_length = distance_to_conflict_zones - negotiation_length  #  length between negotiation zone and first conflict zone
safe_distance_after = 5  # minimum distance between vehicles after the intersection
reach_conflict = 4  # distance from the beginning of the intersection to the first conflict zone
dummy_time = 100000  # default variable ensuring vehicles remain controlled by the script until their exit condition is met

# coordinate: start position of the intersection boundaries
intersection_start = 10.4
# coordinate: start position of the negotiation zone
negotiation_start = intersection_start + control_length + negotiation_length - reach_conflict
# coordinate: end position of the negotiation zone
negotiation_end = negotiation_start - negotiation_length

# target speeds (m/s) for different vehicle maneuvers
# desired speed inside the intersection for vehicles going straight
straight_intersection_speed = 13.89
# desired speed inside the intersection for vehicles turning
turn_intersection_speed = 6


# conflict zone lengths
# distance to the end of the first conflict zone when going straight
straight_first_length = 3.2
# distance to the beginning of the second conflict zone when going straight
straight_second_entry = 4.8
# distance to the end of the second conflict zone when going straight
straight_second_length = 8
# distance to the beginning of the third conflict zone when going straight
straight_third_entry = 8.6
# distance to the end of the third conflict zone when going straight
straight_third_length = 16.8
# distance to the end of the first conflict zone when turning right
right_length = 4.74
# distance to the end of the first conflict zone when turning left
left_first_length = 5.53
# distance to the beginning of the second conflict zone when turning left
left_second_entry = 7.52
# distance to the end of the second conflict zone when turning left
left_second_length = 11.25

# SUMO edge IDs for incoming and outgoing lanes (e=east, n=north, w=west, s=south, j=junction, sr=straight/right, l=left)
east_in = 'etoj'
north_in = 'ntoj'
west_in = 'wtoj'
south_in = 'stoj'
east_in_sr = 'etoj_0'
north_in_sr = 'ntoj_0'
west_in_sr = 'wtoj_0'
south_in_sr = 'stoj_0'
east_in_l = 'etoj_1'
north_in_l = 'ntoj_1'
west_in_l = 'wtoj_1'
south_in_l = 'stoj_1'
east_out = 'jtoe'
north_out = 'jton'
west_out = 'jtow'
south_out = 'jtos'

# data structures to track exiting vehicles and crossing speeds for each direction
exit_east = []
exit_north = []
exit_west = []
exit_south = []
cross_speed_east = {}
cross_speed_north = {}
cross_speed_west = {}
cross_speed_south = {}

# last vehicle's entry time to the negotiation zone for each direction
last_initial_east = None
last_initial_north = None
last_initial_west = None
last_initial_south = None
last_initial_east_left = None
last_initial_north_left = None
last_initial_west_left = None
last_initial_south_left = None

# last vehicle's profile for each direction from negotiation zone to the first conflict zone
last_position_east = None
last_position_north = None
last_position_west = None
last_position_south = None
last_position_east_left = None
last_position_north_left = None
last_position_west_left = None
last_position_south_left = None

# last vehicle's entry time to the first conflict zone for each direction
last_entry_east = 0
last_entry_north = 0
last_entry_west = 0
last_entry_south = 0
last_entry_east_left = 0
last_entry_north_left = 0
last_entry_west_left = 0
last_entry_south_left = 0

# last vehicle's length
last_veh_length_east = 0
last_veh_length_north = 0
last_veh_length_west = 0
last_veh_length_south = 0
last_veh_length_east_left = 0
last_veh_length_north_left = 0
last_veh_length_west_left = 0
last_veh_length_south_left = 0

# spatial requirements and buffer sizes for specific maneuvers through conflict zones.
# keys represent maneuvers (S = Straight, R = Right, L = Left) and their respective phases 
# as a vehicle crosses multiple conflict zones.
#
# phase mapping:
#  - S1, S2, S3: First, second, and third conflict zones crossed when going straight.
#  - R: The single conflict zone crossed when turning right.
#  - L1, L2: First and second conflict zones crossed when turning left.
#
# metric definitions:
#  - "min_entry_gap": Distance from the intersection entry to the start of this conflict zone.
#  - "min_exit_gap": Distance from the intersection entry to the exit of this conflict zone 
#                    (ensures the entire vehicle has cleared the zone).
#  - "required_gap": Total distance the vehicle travels while entirely within this conflict zone 
#                    (accounts for the vehicle's full length).
check_val = {
    'S1' : {
        "min_entry_gap": 0.0,
        "min_exit_gap": 8.2,
        "required_gap": 8.2,
    },
    'S2' : {
        "min_entry_gap": 4.8,
        "min_exit_gap": 13,
        "required_gap": 8.2,
    },
    'S3' : {
        "min_entry_gap": 8.6,
        "min_exit_gap": 21.8,
        "required_gap": 13.2,
    },
    'R' : {
        "min_entry_gap": 0.0,
        "min_exit_gap": 9.74,
        "required_gap": 9.74,
    },
    'L1' : {
        "min_entry_gap": 0.0,
        "min_exit_gap": 8.83,
        "required_gap": 8.83,
    },
    'L2' : {
        "min_entry_gap": 7.52,
        "min_exit_gap": 16.25,
        "required_gap": 8.73,
    },
}
