# file paths for the SUMO simulation configuration and output results
sumo_cfg = r"sumocfg/roundabout.sumocfg"
output_path = 'results/roundabout.xml'

# SUMO parameters
step_length = "0.01"  # discrete step used by SUMO
simulation_steps = 3000000  # simulation length

# intersection layout parameters
negotiation_length = 10  # length of each negotiation zone
distance_to_conflict_zones = 100  # distance from the beginning of the negotiation zone to the first conflict zone
control_length = distance_to_conflict_zones - negotiation_length  #  length between negotiation zone and intersection
safe_distance_after = 5  # minimum distance between vehicles after the intersection
dummy_time = 100000  # default variable ensuring vehicles remain controlled by the script until their exit condition is met

# coordinate: start position of the intersection boundaries
intersection_start = 15.5
# coordinate: start position of the negotiation zone
negotiation_start = intersection_start + control_length + negotiation_length
# coordinate: end position of the negotiation zone
negotiation_end = negotiation_start - negotiation_length

# desired speed inside the intersection
max_intersection_speed = 6.5

# conflict zone lengths
# distance to the end of the first conflict zone
first_length = 4.44
# distance to the beginning of the second conflict zone
second_entry = 8.34
# distance to the end of the second conflict zone
second_length = 12.6
# distance to the beginning of the third conflict zone
third_entry = 19.44
# distance to the end of the third conflict zone
third_length = 22.68
# distance to the beginning of the fourth conflict zone
fourth_entry = 26.22
# distance to the end of the fourth conflict zone
fourth_length = 30.48
# distance to the beginning of the fifth conflict zone
fifth_entry = 37.32
# distance to the end of the fifth conflict zone
fifth_length = 40.56
# distance to the beginning of the sixth conflict zone
sixth_entry = 44.1
# distance to the end of the sixth conflict zone
sixth_length = 48.36

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
past_length = {}

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
#  - R1, R2: First and second conflict zones crossed (all outgoing directions).
#  - S3, S4: Third and fourth conflict zones crossed (only straight and left directions).
#  - L5, L6: Fifth and sixth conflict zones crossed (only left direction).
#
# metric definitions:
#  - "min_entry_gap": Distance from the intersection entry to the start of this conflict zone.
#  - "min_exit_gap": Distance from the intersection entry to the exit of this conflict zone 
#                    (ensures the entire vehicle has cleared the zone).
#  - "required_gap": Total distance the vehicle travels while entirely within this conflict zone 
#                    (accounts for the vehicle's full length).
check_val = {
    'R1' : {
        "min_entry_gap": 0.0,
        "min_exit_gap": 9.54,
        "required_gap": 9.54,
    },
    'R2' : {
        "min_entry_gap": 8.34,
        "min_exit_gap": 17.75,
        "required_gap": 9.41,
    },
    'S3' : {
        "min_entry_gap": 19.44,
        "min_exit_gap": 27.83,
        "required_gap": 8.39,
    },
    'S4' : {
        "min_entry_gap": 26.22,
        "min_exit_gap": 35.63,
        "required_gap": 9.41,
    },
    'L5' : {
        "min_entry_gap": 37.32,
        "min_exit_gap": 45.71,
        "required_gap": 8.39,
    },
    'L6' : {
        "min_entry_gap": 44.1,
        "min_exit_gap": 53.51,
        "required_gap": 9.41,
    },
}
