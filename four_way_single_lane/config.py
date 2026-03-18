# file paths for the SUMO simulation configuration and output results
sumo_cfg = r"sumocfg/four_way_single_lane.sumocfg"
output_path = 'results/four_way_single_lane.xml'

# SUMO parameters
step_length = "0.01"  # discrete step used by SUMO
simulation_steps = 3000000  # simulation length

# intersection layout parameters
negotiation_length = 10  # length of each negotiation zone
distance_to_conflict_zones = 100  # distance from the beginning of the negotiation zone to the first conflict zone
control_length = distance_to_conflict_zones - negotiation_length  #  length between negotiation zone and first conflict zone
safe_distance_after = 5  # minimum distance between vehicles after the intersection

# coordinate: start position of the intersection boundaries
intersection_start = 7.2
# coordinate: start position of the negotiation zone
negotiation_start = intersection_start + control_length + negotiation_length
# coordinate: end position of the negotiation zone
negotiation_end = negotiation_start - negotiation_length
# coordinate: end position of the intersection boundaries
intersection_end = 7.2

# target speeds (m/s) for different vehicle maneuvers
# desired speed inside the intersection for vehicles going straight
straight_intersection_speed = 13.89
# desired speed inside the intersection for vehicles turning
turn_intersection_speed = 6

# conflict zone lengths
# distance to the end of the first conflict zone when going straight
straight_first_length = 7.2
# distance to the end of the second conflict zone when going straight
straight_second_length = 14.4
# distance to the end of the first conflict zone when going right
right_length = 9.02
# distance to the end of the first conflict zone when going left
left_first_length = 5.93
# distance to the end of the second conflict zone when going left
left_second_length = 8.3
# distance to the end of the third conflict zone when going left
left_third_length = 14.24

# SUMO edge IDs for incoming and outgoing lanes (e=east, n=north, w=west, s=south, j=junction)
east_in = 'etoj'
north_in = 'ntoj'
west_in = 'wtoj'
south_in = 'stoj'
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

# last vehicle's profile for each direction from negotiation zone to the first conflict zone
last_position_east = None
last_position_north = None
last_position_west = None
last_position_south = None

# last vehicle's entry time to the first conflict zone for each direction
last_entry_east = 0
last_entry_north = 0
last_entry_west = 0
last_entry_south = 0

# last vehicle's length
last_veh_length_east = 0
last_veh_length_north = 0
last_veh_length_west = 0
last_veh_length_south = 0

# spatial requirements and buffer sizes for specific maneuvers through conflict zones.
# keys represent maneuvers (S = Straight, R = Right, L = Left) and their respective phases 
# as a vehicle crosses multiple conflict zones.
#
# phase mapping:
#  - S1, S2: First and second conflict zones crossed when going straight.
#  - R: The single conflict zone crossed when turning right.
#  - L1, L2, L3: First, second, and third conflict zones crossed when turning left.
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
        "min_exit_gap": 12.2,
        "required_gap": 12.2,
    },
    'S2' : {
        "min_entry_gap": 7.2,
        "min_exit_gap": 19.4,
        "required_gap": 12.2,
    },
    'R' : {
        "min_entry_gap": 0.0,
        "min_exit_gap": 14.0,
        "required_gap": 14.0,
    },
    'L1' : {
        "min_entry_gap": 0.0,
        "min_exit_gap": 11.23,
        "required_gap": 11.23,
    },
    'L2' : {
        "min_entry_gap": 5.93,
        "min_exit_gap": 13.35,
        "required_gap": 7.42,
    },
    'L3' : {
        "min_entry_gap": 8.3,
        "min_exit_gap": 19.24,
        "required_gap": 10.94,
    },
}
