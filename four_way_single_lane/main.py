from typing import Dict, List, Optional, Any
import argparse

parser = argparse.ArgumentParser(description="Run SUMO simulation with a custom GUI path.")
parser.add_argument(
	"--sumo_gui", 
	type=str, 
	default="sumo-gui", 
	help="Full path to the sumo-gui executable"
)
args = parser.parse_args()
sumo_gui = args.sumo_gui

import four_way_single_lane.config as config
from four_way_single_lane.config import (
    sumo_cfg,
    output_path,
    step_length,
    simulation_steps,
    negotiation_start,
    intersection_start,
    negotiation_end,
    intersection_end,
    straight_intersection_speed,
    turn_intersection_speed,
    negotiation_length,
    control_length,
    east_in,
    north_in,
    west_in,
    south_in,
    east_out,
    north_out,
    west_out,
    south_out,
    cross_speed_east,
    cross_speed_north,
    cross_speed_west,
    cross_speed_south,
    exit_east,
    exit_north,
    exit_west,
    exit_south,
    right_length,
    straight_first_length,
    straight_second_length,
    left_first_length,
    left_second_length,
    left_third_length,
    last_initial_east,
    last_initial_north,
    last_initial_west,
    last_initial_south,
    last_position_east,
    last_position_north,
    last_position_west,
    last_position_south,
    last_entry_east,
    last_entry_north,
    last_entry_west,
    last_entry_south,
    last_veh_length_east,
    last_veh_length_north,
    last_veh_length_west,
    last_veh_length_south,
)
import traci
import bisect

import helper
helper.config = config

from helper import (
    before_check,
    after_check,
    do_check,
    compute_entry_change_speed,
    profile_update_accel,
    profile_update_decel,
    logger,
)


# ------------------------------------------------------------------------------
# Initialize SUMO and define the control/intersection polygons
# ------------------------------------------------------------------------------
sumo_cmd: List[str] = [
    sumo_gui,
    "-c",
    sumo_cfg,
    "--device.emissions.probability",
    "1",
    "--tripinfo-output",
    output_path,
    "--step-length",
    step_length,
]
traci.start(sumo_cmd)

# ------------------------------------------------------------------------------
# Create polygons for conflict and negotiation zones for visualization
# ------------------------------------------------------------------------------
traci.polygon.add(
    "sub1",
    [
        (-intersection_start, -intersection_start),
        (0.0, -intersection_start),
        (0.0, 0.0),
        (-intersection_start, 0.0),
    ],
    color=(0, 150, 200, 128),
    fill=True,
    layer=5,
)
traci.polygon.add(
    "sub2",
    [
        (0.0, -intersection_start),
        (0.0, 0.0),
        (intersection_start, 0.0),
        (intersection_start, -intersection_start),
    ],
    color=(255, 255, 0, 128),
    fill=True,
    layer=5,
)
traci.polygon.add(
    "sub3",
    [
        (0.0, 0.0),
        (0.0, intersection_start),
        (intersection_start, intersection_start),
        (intersection_start, 0.0),
    ],
    color=(0, 100, 0, 128),
    fill=True,
    layer=5,
)
traci.polygon.add(
    "sub4",
    [
        (-intersection_start, 0.0),
        (0.0, 0.0),
        (0.0, intersection_start),
        (-intersection_start, intersection_start),
    ],
    color=(128, 0, 128, 128),
    fill=True,
    layer=5,
)
traci.polygon.add(
    "neg1",
    [
        ((intersection_start+100), 0.0),
        ((intersection_start+100), 3.2),
        ((intersection_start+100)-negotiation_length, 3.2),
        ((intersection_start+100)-negotiation_length, 0.0),
    ],
    color=(255, 0, 0, 128),
    fill=True,
    layer=5,
)
traci.polygon.add(
    "neg2",
    [
        (0.0, (intersection_start+100)),
        (-3.2, (intersection_start+100)),
        (-3.2, (intersection_start+100) - negotiation_length),
        (0.0, (intersection_start+100) - negotiation_length),
    ],
    color=(255, 0, 0, 128),
    fill=True,
    layer=5,
)
traci.polygon.add(
    "neg3",
    [
        (-(intersection_start+100), 0.0),
        (-(intersection_start+100), -3.2),
        (-(intersection_start+100) + negotiation_length, -3.2),
        (-(intersection_start+100) + negotiation_length, 0.0),
    ],
    color=(255, 0, 0, 128),
    fill=True,
    layer=5,
)
traci.polygon.add(
    "neg4",
    [
        (0.0, -(intersection_start+100)),
        (3.2, -(intersection_start+100)),
        (3.2, -(intersection_start+100) + negotiation_length),
        (0.0, -(intersection_start+100) + negotiation_length),
    ],
    color=(255, 0, 0, 128),
    fill=True,
    layer=5,
)

logger.info("SUMO simulation started and intersection polygons created.")


# ------------------------------------------------------------------------------
# Initialize data structures for managing vehicle scheduling
# ------------------------------------------------------------------------------
sub_1_entry: List[float] = [0.]
sub_2_entry: List[float] = [0.]
sub_3_entry: List[float] = [0.]
sub_4_entry: List[float] = [0.]

sub_1_exit: List[float] = [0.]
sub_2_exit: List[float] = [0.]
sub_3_exit: List[float] = [0.]
sub_4_exit: List[float] = [0.]

has_entered: Dict[str, bool] = {}
has_crossed: Dict[str, bool] = {}
algorithm_triggered: Dict[str, bool] = {}
position_eq: Dict[str, Any] = {}
crossing_speed: Dict[str, float] = {}
midway_speed: Dict[str, float] = {}
update_distance: Dict[str, Optional[float]] = {}
update_control: Dict[str, bool] = {}
backup_count: Dict[str, bool] = {}


# ------------------------------------------------------------------------------
# Main simulation loop
# ------------------------------------------------------------------------------
for step in range(simulation_steps):
    traci.simulationStep()
    # Get the IDs of all vehicles in the current step
    vehicle_ids = traci.vehicle.getIDList()
    for vehicle_id in vehicle_ids:
        # Check whether the vehicle still has to cross the intersection
        if not(has_crossed.get(vehicle_id,False)):
            route = traci.vehicle.getRoute(vehicle_id)
            position = traci.vehicle.getPosition(vehicle_id)
            vehicle_length = traci.vehicle.getLength(vehicle_id)

            # Variables to use according to the coming direction
            if route[0]==east_in:
                ref_position = position[0]
                straight = west_out
                right = north_out
                left = south_out
                first_area_entry = sub_3_entry
                first_area_exit = sub_3_exit
                straight_area_entry = sub_4_entry
                straight_area_exit = sub_4_exit
                left_area1_entry = sub_2_entry
                left_area1_exit = sub_2_exit
                left_area2_entry = sub_1_entry
                left_area2_exit = sub_1_exit
                update_pos = position[0]
                out_pos_first = -position[0]
                out_pos_second = position[1]
                last_entry = last_entry_east
                last_initial = last_initial_east
                last_position = last_position_east
                last_veh_length = last_veh_length_east
                list_exit_straight = exit_west
                cross_speed_dict_straight = cross_speed_west
                list_exit_right = exit_north
                cross_speed_dict_right = cross_speed_north
                list_exit_left = exit_south
                cross_speed_dict_left = cross_speed_south

            elif route[0]==north_in:
                ref_position = position[1]
                straight = south_out
                right = west_out
                left = east_out
                first_area_entry = sub_4_entry
                first_area_exit = sub_4_exit
                straight_area_entry = sub_1_entry
                straight_area_exit = sub_1_exit
                left_area1_entry = sub_3_entry
                left_area1_exit = sub_3_exit
                left_area2_entry = sub_2_entry
                left_area2_exit = sub_2_exit
                update_pos = position[1]
                out_pos_first = -position[1]
                out_pos_second = position[0]
                last_entry = last_entry_north
                last_initial = last_initial_north
                last_position = last_position_north
                last_veh_length = last_veh_length_north
                list_exit_straight = exit_south
                cross_speed_dict_straight = cross_speed_south
                list_exit_right = exit_west
                cross_speed_dict_right = cross_speed_west
                list_exit_left = exit_east
                cross_speed_dict_left = cross_speed_east

            elif route[0]==west_in:
                ref_position = -position[0]
                straight = east_out
                right = south_out
                left = north_out
                first_area_entry = sub_1_entry
                first_area_exit = sub_1_exit
                straight_area_entry = sub_2_entry
                straight_area_exit = sub_2_exit
                left_area1_entry = sub_4_entry
                left_area1_exit = sub_4_exit
                left_area2_entry = sub_3_entry
                left_area2_exit = sub_3_exit
                update_pos = -position[0]
                out_pos_first = position[0]
                out_pos_second = position[1]
                last_entry = last_entry_west
                last_initial = last_initial_west
                last_position = last_position_west
                last_veh_length = last_veh_length_west
                list_exit_straight = exit_east
                cross_speed_dict_straight = cross_speed_east
                list_exit_right = exit_south
                cross_speed_dict_right = cross_speed_south
                list_exit_left = exit_north
                cross_speed_dict_left = cross_speed_north

            elif route[0]==south_in:
                ref_position = -position[1]
                straight = north_out
                right = east_out
                left = west_out
                first_area_entry = sub_2_entry
                first_area_exit = sub_2_exit
                straight_area_entry = sub_3_entry
                straight_area_exit = sub_3_exit
                left_area1_entry = sub_1_entry
                left_area1_exit = sub_1_exit
                left_area2_entry = sub_4_entry
                left_area2_exit = sub_4_exit
                update_pos = -position[1]
                out_pos_first = position[1]
                out_pos_second = position[0]
                last_entry = last_entry_south
                last_initial = last_initial_south
                last_position = last_position_south
                last_veh_length = last_veh_length_south
                list_exit_straight = exit_north
                cross_speed_dict_straight = cross_speed_north
                list_exit_right = exit_east
                cross_speed_dict_right = cross_speed_east
                list_exit_left = exit_west
                cross_speed_dict_left = cross_speed_west

            else:
                raise Exception("Route not recognized")

            # ------------------------------------------------------------------------------
            # Scheduling of the EGO vehicle
            # ------------------------------------------------------------------------------
            # If the vehicle still has to be scheduled, we schedule it 
            if not(algorithm_triggered.get(vehicle_id,False)):
                # If the vehicle has entered the negotiation zone, we schedule it
                if ref_position <= negotiation_start:
                    # If the scenario is currently in backup mode, we do not schedule it
                    if len(backup_count) > 0:
                        backup_count[vehicle_id] = True
                        algorithm_triggered[vehicle_id] = True
                    # otherwise, we schedule it
                    else:
                        initial_time = traci.simulation.getTime()
                        initial_speed = traci.vehicle.getSpeed(vehicle_id)
                        max_accel = traci.vehicle.getAccel(vehicle_id)
                        decel = traci.vehicle.getDecel(vehicle_id)
                        speed_mode = 54
                        traci.vehicle.setSpeedMode(vehicle_id, speed_mode)
                        traci.vehicle.setSpeed(vehicle_id, initial_speed)

                        # ------------------------------------------------------------------------------
                        # Scheduling of the EGO vehicle according to its trajectory (straight, right, left)
                        # ------------------------------------------------------------------------------
                        # Check if the vehicle will go straight
                        if route[1] == straight:
                            crossing_speed[vehicle_id] = straight_intersection_speed
                            midway_speed[vehicle_id] = None
                            update_distance[vehicle_id] = 0
                            # Compute the proposed entry time at intersection and mobility profile
                            entry_time,position_eq[vehicle_id] = compute_entry_change_speed(
                                initial_speed,
                                straight_intersection_speed,
                                control_length,
                                negotiation_length,
                                initial_time,
                                mode="accel",
                                decel=None,
                                max_accel=max_accel
                            )
                            logger.info("Entry time of " + str(vehicle_id) + " assuming no collisions: " + str(entry_time))
                            # ------------------------------------------------------------------------------
                            # The EGO and the controller have to agree about EGO mobility profile through one or more interactions
                            # ------------------------------------------------------------------------------
                            algorithm_triggered[vehicle_id] = False
                            redo = True
                            while redo:
                                redo = False
                                initial_entry = entry_time
                                if last_entry:
                                    # Check for collisions before the intersection (internal to the controller)
                                    entry_time = before_check(
                                        entry_time,
                                        initial_time,
                                        position_eq[vehicle_id],
                                        last_initial,
                                        last_position,
                                        last_veh_length
                                    )
                                # Perform the remaining checks for collisions (internal to the controller)
                                while not algorithm_triggered[vehicle_id]:
                                    algorithm_triggered[vehicle_id] = True
                                    temp_entry = entry_time
                                    # Check for collisions on the first conflict zone
                                    entry_time = do_check(
                                        first_area_entry,
                                        first_area_exit,
                                        entry_time,
                                        crossing_speed[vehicle_id],
                                        mode="S1")
                                    # Check for collisions on the second conflict zone
                                    entry_time = do_check(
                                        straight_area_entry,
                                        straight_area_exit,
                                        entry_time,
                                        crossing_speed[vehicle_id],
                                        mode="S2"
                                    )
                                    if len(list_exit_straight) > 0:
                                        # Check for collisions after the intersection
                                        entry_time = after_check(
                                            entry_time,
                                            crossing_speed[vehicle_id],
                                            list_exit_straight,
                                            cross_speed_dict_straight,
                                            decel,
                                            mode = "S2"
                                        )
                                    if temp_entry != entry_time:
                                        algorithm_triggered[vehicle_id] = False
                                # If a new entry time is required by any of the checks, the EGO has to compute a new profile and the controller checks for collisions again
                                if initial_entry != entry_time:
                                    redo = True
                                    algorithm_triggered[vehicle_id] = False
                                    crossing_speed[vehicle_id],midway_speed[vehicle_id],update_distance[vehicle_id],position_eq[vehicle_id] = profile_update_accel(
                                        entry_time,
                                        initial_speed,
                                        straight_intersection_speed,
                                        max_accel,
                                        decel,
                                        control_length,
                                        negotiation_length,
                                        initial_time
                                    )

                            # Store the conflict zone entry / exit times and other parameters
                            bisect.insort(first_area_entry, entry_time)
                            bisect.insort(first_area_exit, entry_time + (straight_first_length + vehicle_length) / crossing_speed[vehicle_id])
                            bisect.insort(straight_area_entry,entry_time + straight_first_length / crossing_speed[vehicle_id])
                            bisect.insort(straight_area_exit,entry_time + (straight_second_length + vehicle_length) / crossing_speed[vehicle_id])
                            bisect.insort(list_exit_straight,entry_time + (straight_second_length + vehicle_length) / crossing_speed[vehicle_id])
                            cross_speed_dict_straight[entry_time + (straight_second_length + vehicle_length) / crossing_speed[vehicle_id]] = crossing_speed[vehicle_id]
                            logger.info("Entry time of " + str(vehicle_id) + " at the end of the procedure: " + str(entry_time))
                            logger.info("----------------------------------------")

                        # The same procedure is performed here for vehicles turning at the intersection
                        else:
                            # Compute the proposed entry time at intersection and mobility profile
                            # if the 'initial_speed' is lower than 'turn_intersection_speed', the EGO proposes a mobility profile with a constant speed or with an acceleration phase
                            if initial_speed <= turn_intersection_speed:
                                crossing_speed[vehicle_id] = turn_intersection_speed
                                midway_speed[vehicle_id] = None
                                update_distance[vehicle_id] = 0
                                entry_time,position_eq[vehicle_id] = compute_entry_change_speed(
                                    initial_speed,
                                    turn_intersection_speed,
                                    control_length,
                                    negotiation_length,
                                    initial_time,
                                    mode="accel",
                                    decel=None,
                                    max_accel=max_accel
                                )
                                logger.info("Entry time of " + str(vehicle_id) + " assuming no collisions: " + str(entry_time))
                            # otherwise, the EGO proposes a mobility profile with a deceleration phase
                            else:
                                crossing_speed[vehicle_id] = turn_intersection_speed
                                midway_speed[vehicle_id] = None
                                update_distance[vehicle_id] = control_length - (initial_speed**2 - crossing_speed[vehicle_id]**2) / (2 * decel)
                                entry_time,position_eq[vehicle_id] = compute_entry_change_speed(
                                    initial_speed,
                                    turn_intersection_speed,
                                    control_length,
                                    negotiation_length,
                                    initial_time,
                                    mode="decel",
                                    decel=decel,
                                    max_accel=None
                                )
                                logger.info("Entry time of " + str(vehicle_id) + " assuming no collisions: " + str(entry_time))
                            # ------------------------------------------------------------------------------
                            # The EGO and the controller have to agree about EGO mobility profile through one or more interactions
                            # ------------------------------------------------------------------------------
                            algorithm_triggered[vehicle_id] = False
                            redo = True
                            while redo:
                                redo = False
                                initial_entry = entry_time
                                if last_entry:
                                    # Check for collisions before the intersection (internal to the controller)
                                    entry_time = before_check(
                                        entry_time,
                                        initial_time,
                                        position_eq[vehicle_id],
                                        last_initial,
                                        last_position,
                                        last_veh_length
                                    )
                                # Perform the remaining checks for collisions (internal to the controller)
                                while not algorithm_triggered[vehicle_id]:
                                    algorithm_triggered[vehicle_id]=True
                                    temp_entry = entry_time
                                    # Check for collisions on the first conflict zone when turning right
                                    if route[1] == right:
                                        entry_time = do_check(
                                            first_area_entry,
                                            first_area_exit,
                                            entry_time,
                                            crossing_speed[vehicle_id],
                                            mode="R"
                                        )
                                    
                                    # Check for collisions on conflict zones when turning left
                                    elif route[1] == left:
                                        # Check for collisions on the first conflict zone when turning left
                                        entry_time = do_check(
                                            first_area_entry,
                                            first_area_exit,
                                            entry_time,
                                            crossing_speed[vehicle_id],
                                            mode="L1"
                                        )
                                        # Check for collisions on the second conflict zone when turning left
                                        entry_time = do_check(
                                            left_area1_entry,
                                            left_area1_exit,
                                            entry_time,
                                            crossing_speed[vehicle_id],
                                            mode="L2"
                                        )
                                        # Check for collisions on the third conflict zone when turning left
                                        entry_time = do_check(
                                            left_area2_entry,
                                            left_area2_exit,
                                            entry_time,
                                            crossing_speed[vehicle_id],
                                            mode="L3"
                                        )

                                    if temp_entry != entry_time:
                                        algorithm_triggered[vehicle_id] = False
                                # If a new entry time is required by any of the checks, the EGO has to compute a new profile and the controller checks for collisions again
                                if initial_entry != entry_time:
                                    redo = True
                                    algorithm_triggered[vehicle_id] = False
                                    if initial_speed <= turn_intersection_speed:
                                        crossing_speed[vehicle_id],midway_speed[vehicle_id],update_distance[vehicle_id],position_eq[vehicle_id] = profile_update_accel(
                                            entry_time,
                                            initial_speed,
                                            turn_intersection_speed,
                                            max_accel,
                                            decel,
                                            control_length,
                                            negotiation_length,
                                            initial_time
                                        )
                                    else:
                                        crossing_speed[vehicle_id],midway_speed[vehicle_id],update_distance[vehicle_id],position_eq[vehicle_id] = profile_update_decel(
                                            entry_time,
                                            initial_speed,
                                            turn_intersection_speed,
                                            max_accel,
                                            decel,
                                            control_length,
                                            negotiation_length,
                                            initial_time
                                        )
                            
                            # Store the conflict zone entry / exit times and other parameters when turning right
                            if route[1]==right:
                                bisect.insort(first_area_entry, entry_time)
                                bisect.insort(first_area_exit, entry_time + (right_length + vehicle_length) / crossing_speed[vehicle_id])
                                bisect.insort(list_exit_right,entry_time + (right_length + vehicle_length) / crossing_speed[vehicle_id])
                                cross_speed_dict_right[entry_time + (right_length+vehicle_length)/crossing_speed[vehicle_id]] = crossing_speed[vehicle_id]
                                logger.info("Entry time of "+str(vehicle_id)+" at the end of the procedure: "+str(entry_time))
                                logger.info("----------------------------------------")

                            # Store the conflict zone entry / exit times and other parameters when turning left
                            elif route[1]==left:
                                bisect.insort(first_area_entry,entry_time)
                                bisect.insort(first_area_exit,entry_time + (left_first_length+vehicle_length*1.06)/crossing_speed[vehicle_id])
                                bisect.insort(left_area1_entry,entry_time + left_first_length/crossing_speed[vehicle_id])
                                bisect.insort(left_area1_exit,entry_time + (left_second_length+vehicle_length*1.01)/crossing_speed[vehicle_id])
                                bisect.insort(left_area2_entry,entry_time + left_second_length/crossing_speed[vehicle_id])
                                bisect.insort(left_area2_exit,entry_time + (left_third_length+vehicle_length)/crossing_speed[vehicle_id])
                                list_exit_left = exit_south if route[0]==east_in else exit_north if route[0]==west_in else exit_west if route[0]==south_in else exit_east
                                bisect.insort(list_exit_left,entry_time + (left_third_length+vehicle_length)/crossing_speed[vehicle_id])
                                cross_speed_dict_left[entry_time + (left_third_length+vehicle_length)/crossing_speed[vehicle_id]] = crossing_speed[vehicle_id]
                                logger.info("Entry time of "+str(vehicle_id)+" at the end of the procedure: "+str(entry_time))
                                logger.info("----------------------------------------")

                        # Store further parameters
                        if route[0] == east_in:
                            last_entry_east = entry_time
                            last_initial_east = initial_time
                            last_position_east = position_eq[vehicle_id]
                            last_veh_length_east = vehicle_length

                        elif route[0] == north_in:
                            last_entry_north = entry_time
                            last_initial_north = initial_time
                            last_position_north = position_eq[vehicle_id]
                            last_veh_length_north = vehicle_length

                        elif route[0] == west_in:
                            last_entry_west = entry_time
                            last_initial_west = initial_time
                            last_position_west = position_eq[vehicle_id]
                            last_veh_length_west = vehicle_length

                        elif route[0] == south_in:
                            last_entry_south = entry_time
                            last_initial_south = initial_time
                            last_position_south = position_eq[vehicle_id]
                            last_veh_length_south = vehicle_length

                        else:
                            raise Exception("Route not recognized")

                        update_control[vehicle_id] = True

                        # if the EGO vehicle foresees to have a speed below 3 m/s, the simulation enters in backup mode
                        if midway_speed[vehicle_id] is not None and midway_speed[vehicle_id] < 3:
                            logger.warning('Congestion detected ---> Backup mode')
                            backup_count[vehicle_id] = True
                            update_control[vehicle_id] = False
                            traci.vehicle.setSpeed(vehicle_id, -1)  # remove constraints on vehicle's speed
                            speed_mode_default = 31
                            traci.vehicle.setSpeedMode(vehicle_id, speed_mode_default)
                            # print('Speed is too low')
                            # print(midway_speed[vehicle_id])
                            # print(step)
                            logger.info('------------------------------')

            # Trigger SUMO if the vehicle mobility profile requires any updates 
            if update_control.get(vehicle_id, False):
                if midway_speed[vehicle_id]!=None:
                    if update_pos<=negotiation_end:
                        traci.vehicle.setSpeed(vehicle_id,midway_speed[vehicle_id])
                        midway_speed[vehicle_id] = None
                if update_pos<=negotiation_end-update_distance[vehicle_id]:
                    update_control[vehicle_id] = False
                    traci.vehicle.setSpeed(vehicle_id,crossing_speed[vehicle_id])

            # When the vehicle exits the last conflict zone, the control is given back to SUMO
            if out_pos_first>intersection_end+vehicle_length or abs(out_pos_second)>intersection_end+vehicle_length:
                has_crossed[vehicle_id] = True
                traci.vehicle.setSpeed(vehicle_id, -1)  # remove constraints on vehicle's speed
                speed_mode_default = 31
                traci.vehicle.setSpeedMode(vehicle_id, speed_mode_default)
                if vehicle_id in backup_count:
                    del backup_count[vehicle_id]

traci.close()