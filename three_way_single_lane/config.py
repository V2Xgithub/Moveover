# file paths for the SUMO simulation configuration and output results
sumo_cfg = r"sumocfg/three_way_single_lane.sumocfg"
output_path = 'results/three_way_single_lane.xml'

# SUMO parameters
step_length = "0.01"  # discrete step used by SUMO
simulation_steps = 3000000  # simulation length

# intersection layout parameters
negotiation_length = 10  # length of each negotiation zone
distance_to_conflict_zones = 103.6  # distance from the beginning of the negotiation zone to the first conflict zone
control_length = distance_to_conflict_zones - negotiation_length  #  length between negotiation zone and first conflict zone
safe_distance_after = 5  # minimum distance between vehicles after the intersection
reach_conflict = 3.6  # distance from the beginning of the intersection to the first conflict zone

# coordinate: start position of the intersection boundaries
intersection_start = 7.2
# coordinate: start position of the negotiation zone
negotiation_start = intersection_start + control_length + negotiation_length - reach_conflict
# coordinate: end position of the negotiation zone
negotiation_end = negotiation_start - negotiation_length
# coordinate: end position of the last traversed conflict zone for each direction
intersection_end_east = 7.2
intersection_end_south = 7.2
intersection_end_west = 3.6

# target speeds (m/s) for different vehicle maneuvers
# desired speed inside the intersection for vehicles going straight
straight_intersection_speed = 13.89
# desired speed inside the intersection for vehicles turning
turn_intersection_speed = 6

# conflict zone lengths
# distance to the end of the first conflict zone when going straight from east road
straight_east_length = 7.2
# distance to the end of the first conflict zone when going straight from west road
straight_west_first_length = 3.6
# distance to the end of the second conflict zone when going straight from west road
straight_west_second_length = 10.8
# distance to the end of the first conflict zone when going right
right_length = 5.15
# distance to the end of the first conflict zone when going left
left_first_length = 2.18
# distance to the end of the second conflict zone when going left
left_second_length = 4.49
# distance to the end of the third conflict zone when going left from south road
left_south_third_length = 6.7
# distance to the end of the third conflict zone when going left from east road
left_east_third_length = 10.39

# SUMO edge IDs for incoming and outgoing lanes (e=east, n=north, w=west, s=south, j=junction)
east_in = 'etoj'
west_in = 'wtoj'
south_in = 'stoj'
east_out = 'jtoe'
west_out = 'jtow'
south_out = 'jtos'

# data structures to track exiting vehicles and crossing speeds for each direction
exit_east = []
exit_west = []
exit_south = []
cross_speed_east = {}
cross_speed_west = {}
cross_speed_south = {}
past_length = {}

# last vehicle's entry time to the negotiation zone for each direction
last_initial_east = None
last_initial_west = None
last_initial_south = None

# last vehicle's profile for each direction from negotiation zone to the first conflict zone
last_position_east = None
last_position_west = None
last_position_south = None

# last vehicle's entry time to the first conflict zone for each direction
last_entry_east = 0
last_entry_west = 0
last_entry_south = 0

# last vehicle's length
last_veh_length_east = 0
last_veh_length_west = 0
last_veh_length_south = 0

# spatial requirements and buffer sizes for specific maneuvers through conflict zones.
# keys represent maneuvers (S = Straight, R = Right, L = Left) and their respective phases 
# as a vehicle crosses multiple conflict zones.
#
# phase mapping:
#  - S1_east / S1_west, S2: First and second conflict zones crossed when going straight.
#    The topology of the first onflict zone depends on the incoming direction. 
#  - R: The single conflict zone crossed when turning right.
#  - L1, L2, L3_south / L3_east: First, second, and third conflict zones crossed when turning left.
#    The topology of the third conflict zone depends on the incoming direction.
# metric definitions:
#  - "min_entry_gap": Distance from the intersection entry to the start of this conflict zone.
#  - "min_exit_gap": Distance from the intersection entry to the exit of this conflict zone 
#                    (ensures the entire vehicle has cleared the zone).
#  - "required_gap": Total distance the vehicle travels while entirely within this conflict zone 
#                    (accounts for the vehicle's full length).
check_val = {
    'S1_east' : {
        "min_entry_gap": 0.0,
        "min_exit_gap": 12.2,
        "required_gap": 12.2,
    },
    'S1_west' : {
        "min_entry_gap": 0.0,
        "min_exit_gap": 8.6,
        "required_gap": 8.6,
    },
    'S2' : {
        "min_entry_gap": 3.6,
        "min_exit_gap": 15.8,
        "required_gap": 12.2,
    },
    'R' : {
        "min_entry_gap": 0.0,
        "min_exit_gap": 10.15,
        "required_gap": 10.15,
    },
    'L1' : {
        "min_entry_gap": 0.0,
        "min_exit_gap": 7.48,
        "required_gap": 7.48,
    },
    'L2' : {
        "min_entry_gap": 2.18,
        "min_exit_gap": 9.54,
        "required_gap": 7.36,
    },
    'L3_south' : {
        "min_entry_gap": 4.49,
        "min_exit_gap": 11.7,
        "required_gap": 7.21,
    },
    'L3_east' : {
        "min_entry_gap": 4.49,
        "min_exit_gap": 15.39,
        "required_gap": 10.9,
    },
}
