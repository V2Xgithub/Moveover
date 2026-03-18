import logging
import bisect
import math
from typing import List, Dict, Tuple, Optional, Union
import numpy as np

from scipy.optimize import fsolve
from sympy import symbols, Piecewise, Expr


# Configure logger for this module
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Define time symbol for sympy expressions
t = symbols('t')


def do_check(
    sub_entry: List[float],
    sub_exit: List[float],
    entry_time: float,
    crossing_speed: float,
    mode: str,
) -> float:
    """
    Adjust entry time based on scheduling constraints at conflict zones. The binary variable 'scheduled' represents whether the EGO vehicle is successfully scheduled for the considered conflict zone.

    Parameters:
        sub_entry: Entry times of other vehicles.
        sub_exit: Exit times of other vehicles.
        entry_time: Proposed entry time.
        crossing_speed: Speed during crossing.
        mode: Current 'phase' (see config.py).

    Returns:
        Adjusted entry time.
    """
    gaps = config.check_val[mode]
    # If proposed entry time precedes the last scheduled ahead vehicle's exit time, scheduling adjustments may be required
    if entry_time + gaps["min_entry_gap"] / crossing_speed < sub_exit[-1]:
        scheduled = False
        exit_time = entry_time + gaps["min_exit_gap"] / crossing_speed
        # 'i' is the ahead vehicle exiting from the considerd conflict zone right before the EGO proposed entry time
        i = bisect.bisect_left(
            sub_exit, entry_time + gaps["min_entry_gap"] / crossing_speed
        ) - 1
        # If the EGO vehicle exits before the next vehicle entering the conflict zone, the proposed scheduling works
        if exit_time <= sub_entry[i + 1]:
            scheduled = True
        # Otherwise, we look for an available gap to schedule the EGO by looking at intervals between pairs of already scheduled vehicles 
        i += 1
        while not scheduled and i <= len(sub_exit) - 2:
            if (
                sub_entry[i + 1] - sub_exit[i]
                >= gaps["required_gap"] / crossing_speed
            ):
                entry_time = sub_exit[i] - gaps["min_entry_gap"] / crossing_speed
                scheduled = True
            i += 1
        # Finally, if no gap is found, the EGO is scheduled to cross the considered conflict zone as the last vehicle in the queue
        if not scheduled:
            entry_time = sub_exit[-1] - gaps["min_entry_gap"] / crossing_speed
    return entry_time


def before_check(
    entry_time: float,
    initial_time: float,
    pos_eq: Expr,
    last_initial: float,
    last_pos: Expr,
    last_veh: float,
) -> float:
    """
    Detect and adjust for potential collisions before the intersection.

    Parameters:
        entry_time: Proposed entry time.
        initial_time: EGO vehicle negotiation zone entry time.
        pos_eq: Equation describing the EGO vehicle position profile up to the first conflict zone.
        last_initial: Ahead vehicle negotiation zone entry time.
        last_pos: Equation describing the ahead vehicle position profile up to the first conflict zone.
        last veh: Ahead vehicle length.

    Returns:
        Adjusted entry time.
    """

    time = initial_time
    min_distance = 2 + last_veh
    collision = False
    time_ahead = time - last_initial  # time elapsed since ahead vehicle entered the negotiation zone
    time_ego = time - initial_time  # time elapsed since EGO vehicle entered the negotiation zone
    position_ahead = last_pos.subs(t, time_ahead) - last_veh  # ahead vehicle position at 'time_ahead' instant
    position_ego = pos_eq.subs(t, time_ego) # EGO vehicle position at 'time_ego' instant

    # For each time step, verify whether there is a collision between the EGO and the ahead vehicle before any of the two enters the conflict zone
    while position_ahead<=config.distance_to_conflict_zones and position_ego<=config.distance_to_conflict_zones-2:
        current_distance = position_ahead - position_ego
        if current_distance < 2:
            collision = True
            # Record the time instant and EGO position when the (possible) collision occurs and the two vehicles are the closest possible
            if(current_distance<=min_distance):
                min_distance = current_distance
                min_time_ego = time
                min_pos_ego = position_ego
        time += 0.1  # each time step lasts 0.1 s
        time_ahead = time - last_initial
        time_ego = time - initial_time
        position_ahead = last_pos.subs(t, time_ahead) - last_veh
        position_ego = pos_eq.subs(t, time_ego)
    # Compute the entry time increase to avoid the collision
    if collision:
        entry_time_increase = compute_increase(
            min_time_ego,
            initial_time,
            min_pos_ego,
            last_initial,
            last_pos,
            last_veh
        )
        entry_time += entry_time_increase
    return entry_time

def compute_increase(
    collision_time: float,
    initial_time: float,
    ego_position: float,
    last_initial: float,
    last_pos: Expr,
    last_veh: float,
) -> float:
    """
    Compute the entry time increase to avoid the collision before the intersection.

    Parameters:
        collision_time: Collision time instant.
        initial_time: EGO vehicle negotiation zone entry time.
        ego_position: EGO vehicle position when the collision occur.
        last_initial: Ahead vehicle negotiation zone entry time.
        last_pos: Equation describing the ahead vehicle position profile up to the first conflict zone.
        last veh: Ahead vehicle length.

    Returns:
        Entry time increase.
    """
    
    find = False
    time = initial_time
    time_ahead = time - initial_time
    position_ahead = last_pos.subs(t,time_ahead) - last_veh
    buffer = 1.5  # this buffer helps to handle discrete time steps
    # Find the time instant in which the ahead vehicle position results to be two meters ahead of the marked position of the EGO vehicle 'ego_position', corresponding to the position when the collision occurs.
    # After that, determine how much time the EGO needs to be delayed for entering the first conflict zone, aiming to avoid that potential collision.
    while position_ahead<=config.distance_to_conflict_zones + buffer and not find:
        if position_ahead-2>=ego_position:
            time_increase = math.ceil(time-collision_time)
            find = True
        time += 0.1
        time_ahead = time - last_initial
        position_ahead = last_pos.subs(t,time_ahead) - last_veh
    return time_increase


def after_check(
    entry_time: float,
    crossing_speed: float,
    exit_time: List[float],
    past_speed: Dict[float, float],
    decel: float,
    mode: str,
) -> float:
    """
    Detect and adjust for potential collision after the intersection.

    Parameters:
        entry_time: Proposed EGO vehicle entry time.
        crossing_speed: EGO vehicle speed during crossing.
        exit_time: Exit times of other vehicles.
        past_speed: Dictionary mapping from ahead vehicles exit time to ahead vehicles speed.
        decel: EGO vehicle deceleration rate.
        mode: Current 'phase' (see config.py).

    Returns:
        Possibly adjusted entry time.
    """
    
    gaps = config.check_val[mode]
    # Locate ahead exiting vehicle index for comparison
    idx = bisect.bisect_right(exit_time, entry_time + gaps["min_entry_gap"] / crossing_speed) - 1
    key_time = exit_time[idx]
    # If the EGO vehicle is faster than ahead exiting vehicle after the last conflict zone and this results in a collision, delay the EGO
    if (
        crossing_speed >= past_speed[key_time]
        and entry_time <= key_time + 1
    ):
        dbrake, dafter = compute_gap(
            entry_time,
            crossing_speed,
            key_time,
            past_speed[key_time],
            decel,
            mode = mode,
        )
        if round(dafter - dbrake, 3) < config.safe_distance_after:
            entry_time = avoid_collision(
                entry_time,
                crossing_speed,
                key_time,
                past_speed[key_time],
                dbrake,
                dafter,
                mode = mode,
            )
    return entry_time


def compute_gap(
    entry_time: float,
    crossing_speed: float,
    exit_time: float,
    past_speed: float,
    decel: float,
    mode: str,
) -> Tuple[float, float]:
    """
    Compute braking and post-braking distances to evaluate vehicle gap.
    
    Parameters:
        entry_time: Proposed EGO vehicle entry time.
        crossing_speed: EGO vehicle speed during crossing.
        exit_time: Exit times of other vehicles.
        past_speed: Dictionary mapping from ahead vehicles exit time to ahead vehicles speed.
        decel: EGO vehicle deceleration rate.
        mode: Current 'phase' (see config.py).

    Returns:
        Tuple containing (dbrake, dafter), where: 
         * 'dbrake' is the distance traveled by the EGO after its braking and 
         * 'dafter' is the distance traveled by the ahead after EGO has completed braking.
    """
    gaps = config.check_val[mode]
    # Remaining distance when ahead vehicle exits
    dleft = gaps["min_exit_gap"] - (exit_time - entry_time) * crossing_speed
    t_gap = dleft / crossing_speed
    # Distance covered by ahead vehicle in t_gap
    d_cover = t_gap * past_speed
    # Distance required to brake from crossing_speed to past_speed
    dbrake = (crossing_speed**2 - past_speed**2) / (2 * decel)
    tbrake = (crossing_speed - past_speed) / decel
    # Distance that the ahead vehicle travels after current starts braking
    dafter = d_cover + tbrake * past_speed - 5
    return dbrake, dafter


def avoid_collision(
    entry_time: float,
    crossing_speed: float,
    exit_time: float,
    past_speed: float,
    dbrake: float,
    dafter: float,
    mode: str,
) -> float:
    """
    Adjust entry time to avoid collision after EGO vehicle braking event.
    
    Parameters:
        entry_time: Proposed EGO vehicle entry time.
        crossing_speed: EGO vehicle speed during crossing.
        exit_time: Exit times of other vehicles.
        past_speed: Dictionary mapping from ahead vehicles exit time to ahead vehicles speed.
        dbrake: Braking distance of EGO vehicle.
        dafter: Distance that the ahead vehicle travels after EGO braking.
        mode: Current 'phase' (see config.py).

    Returns:
        New entry time.
    """
    gaps = config.check_val[mode]
    # Compute leftover distance after past exits
    dleft = gaps["min_exit_gap"] - (exit_time - entry_time) * crossing_speed
    t_gap = dleft / crossing_speed
    d_cover = t_gap * past_speed
    # Extra buffer needed
    buffer = (dbrake + config.safe_distance_after) - dafter
    # Time until safe gap
    t_total = (d_cover + buffer) / past_speed
    new_dleft = t_total * crossing_speed
    # Recompute entry_time based on new leftover
    entry_time = exit_time - (gaps["min_exit_gap"] - new_dleft) / crossing_speed
    return entry_time



def compute_entry_change_speed(
    initial_speed: float,
    intersection_speed: float,
    control_length: float,
    negotiation_length: float,
    initial_time: float,
    mode: str,
    decel: Optional[float],
    max_accel: Optional[float],
) -> Tuple[float, Expr]:
    """
    Compute entry time and position equation for speed change at entry.

    Parameters:
        initial_speed: EGO vehicle starting speed.
        intersection_speed: EGO vehicle target speed at intersection.
        control_length: Length between negotiation zone and first conflict zone.
        negotiation_length: Length of each negotiation zone
        initial_time: EGO vehicle negotiation zone entry time.
        mode: 'decel' or 'accel', for deceleration or acceleration, respectively.
        decel: EGO vehicle deceleration rate.
        max_accel: EGO vehicle acceleration rate.

    Returns:
        Tuple of (entry_time, position equation Expr) reprenting EGO vehicle profile.
    """
    negotiation_offset = negotiation_length / initial_speed  # time needed to cross the negotiation zone
    # t1: time needed to complete acceleration / deceleration
    # d1: distance needed to complete acceleration / deceleration
    if mode == "decel":
        t1 = (initial_speed - intersection_speed) / decel
        d1 = (initial_speed ** 2 - intersection_speed ** 2) / (2 * decel)
    else:
        t1 = (intersection_speed - initial_speed) / max_accel
        d1 = initial_speed * t1 + 0.5 * max_accel * t1 ** 2
    
    # t2: time in which the EGO vehicle proceeds at constant speed (depending on acceleration / deceleration)
    # d2: distance traveled by the EGO vehicle at constant speed (depending on acceleration / deceleration)
    d2 = control_length - d1
    t2 = d2 / initial_speed if mode=="decel" else d2 / intersection_speed
    entry_time = initial_time + negotiation_offset + t1 + t2
    position_eq = (
        position_entry_decel(initial_speed, intersection_speed, decel, control_length, negotiation_length)
        if mode == "decel"
        else position_entry_accel(initial_speed, intersection_speed, max_accel, negotiation_offset)
    )
    return entry_time, position_eq


def profile_update_accel(
    entry_time: float,
    initial_speed: float,
    intersection_speed: float,
    max_accel: float,
    decel: float,
    control_length: float,
    negotiation_length: float,
    initial_time: float,
) -> tuple[Union[int, float, np.ndarray], Union[np.ndarray, None], Union[float, np.ndarray, None], Expr]:
    """
    Update mobility profile for acceleration case.
    
    Parameters:
        entry_time: Proposed EGO vehicle entry time.
        initial_speed: EGO vehicle starting speed.
        intersection_speed: EGO vehicle target speed at intersection.
        max_accel: EGO vehicle acceleration rate.
        decel: EGO vehicle deceleration rate.
        control_length: Length between negotiation zone and first conflict zone.
        negotiation_length: Length of each negotiation zone
        initial_time: EGO vehicle negotiation zone entry time.
        
    We consider 4 possible profiles:
      - Profile A1. We schedule an acceleration phase aiming at having a speed equal to 'intersection_speed' when entering the first conflict zone and according to the required 'entry_time'.
      - Profile A2. We schedule an acceleration phase right before reaching the first conflict zone, aiming at entering the first conflict zone according to the required 'entry_time'.
      - Profile A3. We schedule a deceleration phase right after exiting the negotiation zone, aiming at entering the first conflict zone according to the required 'entry_time'.
      - Profile A4. We schedule a deceleration phase right after exiting the negotiation zone, aiming at entering the first conflict zone according to the required 'entry_time' and ensuring that the speed when reaching and crossing the intersection is above 6 m/s.

    Returns:
        EGO profile as (crossing_speed, midway_speed, update_distance, position_eq)
    """
    negotiation_offset = negotiation_length / initial_speed  # time needed to cross the negotiation zone
    adjusted_time = entry_time - initial_time - negotiation_offset  # EGO vehicle travel time
    t2 = (intersection_speed - initial_speed) / max_accel
    d2 = (intersection_speed ** 2 - initial_speed ** 2) / (2 * max_accel)
    d1 = control_length - d2
    t1 = d1 / initial_speed
    threshold_1 = t1 + t2  # threshold corresponding to the travel time that the vehicle has in case it proceeds at a constant speed and it only accelerates right before the first conflict zone
    threshold_2 = control_length / initial_speed  # threshold corresponding to the travel time that the vehicle has in case it proceeds without any accelerations / decelerations

    # If the entry time in the first conflict zone is below 'threshold_1', apply Profile A1.
    if adjusted_time <= threshold_1:
        crossing_speed = intersection_speed
        midway_speed = None
        update_distance = fsolve(
            compute_change_speed_distance,
            np.array([0]),
            args=(adjusted_time, initial_speed, intersection_speed, max_accel, control_length, "accel"),
        )[0]
        position_eq = position_accel_distance(initial_speed, crossing_speed, max_accel, update_distance+negotiation_length)
    # If the entry time in the first conflict zone is below 'threshold_2', apply Profile A2.
    elif adjusted_time <= threshold_2:
        crossing_speed = fsolve(
            compute_crossing_speed_change,
            np.array([intersection_speed]),
            args=(adjusted_time, initial_speed, max_accel, control_length, "accel"),
        )[0]
        midway_speed = None
        update_distance = control_length - (crossing_speed ** 2 - initial_speed ** 2) / (2 * max_accel)
        position_eq = position_crossing_speed_accel(initial_speed, max_accel, update_distance+negotiation_length, crossing_speed)
    # Otherwise, apply Profile A3.
    else:
        crossing_speed = fsolve(
            compute_crossing_speed_change,
            np.array([initial_speed]),
            args=(adjusted_time, initial_speed, decel, control_length, "decel"),
        )[0]
        midway_speed = None
        update_distance = 0
        position_eq = position_crossing_speed_decel(initial_speed, crossing_speed, decel, negotiation_offset)

    # In any circumstances, if a crossing speed is detected below 6 m/s, apply Profile A4.
    if crossing_speed < 6:
        crossing_speed = 6
        midway_speed = fsolve(
            compute_midway_speed,
            np.array([crossing_speed]),
            args=(adjusted_time, initial_speed, crossing_speed, max_accel, decel, control_length),
        )[0]
        update_distance = control_length - (crossing_speed ** 2 - midway_speed ** 2) / (2 * max_accel)
        position_eq = position_midway_speed(initial_speed, crossing_speed, midway_speed, max_accel, decel, update_distance, negotiation_offset)

    return crossing_speed, midway_speed, update_distance, position_eq


def profile_update_decel(
    entry_time: float,
    initial_speed: float,
    intersection_speed: float,
    max_accel: float,
    decel: float,
    control_length: float,
    negotiation_length: float,
    initial_time: float,
) -> tuple[Union[int, float, np.ndarray], Union[float, np.ndarray, None], Expr]:
    """
    Update mobility profile for deceleration case.
    
    Parameters:
        entry_time: Proposed EGO vehicle entry time.
        initial_speed: EGO vehicle starting speed.
        intersection_speed: EGO vehicle target speed at intersection.
        max_accel: EGO vehicle acceleration rate.
        decel: EGO vehicle deceleration rate.
        control_length: Length between negotiation zone and first conflict zone.
        negotiation_length: Length of each negotiation zone
        initial_time: EGO vehicle negotiation zone entry time.
        
     We consider 3 possible profiles:
      - Profile B1. We schedule a deceleration phase aiming at having a speed equal to 'intersection_speed' when entering the first conflict zone and according to the required 'entry_time'.
      - Profile B2. We schedule a deceleration phase right after reaching the negotiation zone, aiming at entering the first conflict zone according to the required 'entry_time'.
      - Profile B3. We schedule a deceleration phase right after exiting the negotiation zone, aiming at entering the first conflict zone according to the required 'entry_time' and ensuring that the speed when reaching and crossing the intersection is above 6 m/s.

    Returns:
        EGO profile as (crossing_speed, midway_speed, update_distance, position_eq)
    """
    negotiation_offset = negotiation_length / initial_speed
    adjusted_time = entry_time - initial_time - negotiation_offset
    t1 = (initial_speed - intersection_speed) / decel
    d1 = (initial_speed ** 2 - intersection_speed ** 2) / (2 * decel)
    d2 = control_length - d1
    t2 = d2 / intersection_speed
    threshold_1 = t1 + t2  # threshold corresponding to the travel time that the vehicle has in case it proceeds at a constant speed and it only decelerates right after the negotiation zone

    t1 = (initial_speed - 6) / decel
    d1 = (initial_speed**2 - 6**2) / (2*decel)
    d2 = control_length - d1
    t2 = d2 / 6
    threshold_2 = t1 + t2  # threshold corresponding to the travel time that the vehicle has in case it reaches and crosses the intersection at a speed of 6 m/s and it only decelerates right after the negotiation zone

    # If the entry time in the first conflict zone is below 'threshold_1', apply Profile B1.
    if adjusted_time <= threshold_1:
        crossing_speed = intersection_speed
        midway_speed = None
        update_distance = fsolve(
            compute_change_speed_distance,
            np.array([0]),
            args=(adjusted_time, initial_speed, intersection_speed, decel, control_length, "decel"),
        )[0]
        position_eq = position_decel_distance(initial_speed, intersection_speed, decel, update_distance, negotiation_length)
    # If the entry time in the first conflict zone is below 'threshold_2', apply Profile B2.
    elif adjusted_time <= threshold_2:
        crossing_speed = fsolve(
            compute_crossing_speed_change,
            (intersection_speed),
            args=(adjusted_time, initial_speed, decel, control_length, 'decel'))[0]
        midway_speed = None
        update_distance = 0
        position_eq = position_crossing_speed_decel(initial_speed,crossing_speed,decel) 
    # Otherwise, apply Profile B3.
    else:
        crossing_speed = 6
        midway_speed = fsolve(
            compute_midway_speed,
            np.array([intersection_speed]),
            args=(adjusted_time, initial_speed, intersection_speed, max_accel, decel, control_length),
        )[0]
        update_distance = control_length - (intersection_speed ** 2 - midway_speed ** 2) / (2 * max_accel)
        position_eq = position_midway_speed(initial_speed, intersection_speed, midway_speed, max_accel, decel, update_distance, negotiation_offset)

    return crossing_speed, midway_speed, update_distance, position_eq


def compute_change_speed_distance(
    vars: float,
    entry_time: float,
    initial_speed: float,
    crossing_speed: float,
    change_speed: float,
    control_length: float,
    mode: str,
) -> float:
    """
    Compute the time difference for a solver to find the optimal distance to begin changing speed.

    Parameters:
        vars: The unknown variable array (contains `update_distance` in meters).
        entry_time:  Proposed EGO vehicle entry time.
        initial_speed: EGO vehicle starting speed.
        crossing_speed: EGO vehicle speed during crossing.
        change_speed: The rate of speed change (acceleration or deceleration rate).
        control_length: Length between negotiation zone and first conflict zone.
        mode: 'decel' or 'accel', indicating the type of speed change.

    Returns:
        The difference between the required entry time and the computed travel time. A return 
        value of 0 means the `update_distance` correctly schedules the vehicle.
    """
    update_distance = vars
    if mode == "decel":
        start_speed, end_speed = initial_speed, crossing_speed
    else:
        start_speed, end_speed = crossing_speed, initial_speed

    t1 = update_distance / initial_speed
    t2 = (start_speed - end_speed) / change_speed
    d2 = (start_speed ** 2 - end_speed ** 2) / (2 * change_speed)
    d3 = control_length - d2 - update_distance
    t3 = d3 / crossing_speed
    return entry_time - t1 - t2 - t3


def compute_crossing_speed_change(
    vars: float,
    entry_time: float,
    initial_speed: float,
    speed_change: float,
    control_length: float,
    mode: str,
) -> float:
    """
    Compute the time difference for a solver to find the crossing speed ensuring to match 'entry_time' requirement.

    Parameters:
        vars: The unknown variable array (contains `crossing_speed` in m/s).
        entry_time: Proposed EGO vehicle entry time.
        initial_speed: EGO vehicle starting speed.
        speed_change: The rate of speed change (acceleration or deceleration rate).
        control_length: Length between negotiation zone and first conflict zone.
        mode: 'decel' or 'accel', indicating the type of speed change.

    Returns:
        The difference between the required entry time and the computed travel time. A return 
        value of 0 means the `update_distance` correctly schedules the vehicle.
    """
    crossing_speed = vars
    if mode == "decel":
        start_speed, end_speed = initial_speed, crossing_speed
    else:
        start_speed, end_speed = crossing_speed, initial_speed

    t1 = (start_speed - end_speed) / speed_change
    d1 = (start_speed ** 2 - end_speed ** 2) / (2 * speed_change)
    d2 = control_length - d1
    t2 = d2 / end_speed
    return entry_time - t1 - t2


def compute_midway_speed(
    vars: float,
    entry_time: float,
    initial_speed: float,
    crossing_speed: float,
    max_accel: float,
    decel: float,
    control_length: float,
) -> float:
    """
    Compute the time difference for a solver to find an intermediate cruising speed.

    Parameters:
        vars: The unknown variable array (contains `midway_speed` in m/s).
        entry_time: Proposed EGO vehicle entry time.
        initial_speed: EGO vehicle starting speed.
        crossing_speed: EGO vehicle speed during crossing.
        max_accel: EGO vehicle acceleration rate.
        decel: EGO vehicle deceleration rate.
        control_length: Length between negotiation zone and first conflict zone.

    Returns:
        The difference between the required entry time and the computed travel time. A return 
        value of 0 means the `update_distance` correctly schedules the vehicle.
    """
    midway_speed = vars
    t1 = (initial_speed - midway_speed) / decel
    d1 = (initial_speed ** 2 - midway_speed ** 2) / (2 * decel)
    t3 = (crossing_speed - midway_speed) / max_accel
    d3 = (crossing_speed ** 2 - midway_speed ** 2) / (2 * max_accel)
    d2 = control_length - d1 - d3
    t2 = d2 / midway_speed
    return entry_time - t1 - t2 - t3



def position_entry_decel(
    initial_speed: float,
    crossing_speed: float,
    decel: float,
    control_length: float,
    negotiation_length: float,
) -> Expr:
    """
    Generate the position-time equation for a deceleration profile (first proposal).

    Parameters:
        initial_speed: EGO vehicle starting speed.
        crossing_speed: EGO vehicle target speed at intersection.
        decel: EGO vehicle deceleration rate.
        control_length: Length between negotiation zone and first conflict zone.
        negotiation_length: Length of each negotiation zone

    Returns:
        A SymPy Piecewise expression defining the vehicle's position as a function of time (t).
    """
    t1 = ((control_length + negotiation_length) - (initial_speed**2 - crossing_speed**2)/(2*decel)) / initial_speed
    t2 = t1 + (initial_speed - crossing_speed)/decel
    pos1 = initial_speed * t
    pos2 = initial_speed * t1 + initial_speed*(t - t1) - 0.5 * decel * (t - t1)**2
    pos3 = initial_speed * t1 + initial_speed*(t2 - t1) - 0.5 * decel * (t2 - t1)**2 + crossing_speed * (t - t2)
    return Piecewise((pos1, t <= t1), (pos2, (t > t1) & (t <= t2)), (pos3, True))


def position_entry_accel(
    initial_speed: float,
    crossing_speed: float,
    max_accel: float,
    negotiation_offset: float,
) -> Expr:
    """
    Generate the position-time equation for an acceleration profile (first proposal).

    Parameters:
        initial_speed: EGO vehicle starting speed.
        crossing_speed: EGO vehicle speed during crossing.
        max_accel: EGO vehicle acceleration rate.
        negotiation_offset: Time required to cross the negotiation zone.

    Returns:
        A SymPy Piecewise expression defining the vehicle's position as a function of time (t).
    """
    t1 = negotiation_offset
    t2 = t1 + (crossing_speed - initial_speed) / max_accel
    pos1 = initial_speed * t
    pos2 = initial_speed * t1 + initial_speed * (t - t1) + 0.5 * max_accel * (t - t1)**2
    pos3 = initial_speed * t1 + initial_speed * (t2 - t1) + 0.5 * max_accel * (t2 - t1)**2 + crossing_speed * (t - t2)
    return Piecewise((pos1, t <= t1), (pos2, (t > t1) & (t <= t2)), (pos3, True))


def position_accel_distance(
    initial_speed: float,
    crossing_speed: float,
    max_accel: float,
    d1: Union[np.ndarray, float],
) -> Expr:
    """
    Generate the position-time equation for an acceleration phase starting after a specific distance 'd1'.

    Parameters:
        initial_speed: EGO vehicle starting speed.
        crossing_speed: EGO vehicle speed during crossing.
        max_accel: EGO vehicle acceleration rate.
        d1: The distance traveled at the initial speed before acceleration begins.

    Returns:
        A SymPy Piecewise expression defining the vehicle's position as a function of time (t).
    """
    t1 = d1 / initial_speed
    t2 = t1 + (crossing_speed - initial_speed) / max_accel
    pos1 = initial_speed * t
    pos2 = initial_speed * t1 + initial_speed*(t - t1) + 0.5 * max_accel * (t - t1)**2
    pos3 = initial_speed * t1 + initial_speed*(t2 - t1) + 0.5 * max_accel*(t2 - t1)**2 + crossing_speed*(t - t2)
    return Piecewise((pos1, t <= t1), (pos2, (t > t1) & (t <= t2)), (pos3, True))


def position_crossing_speed_accel(
    initial_speed: float,
    max_accel: float,
    d1: Optional[Union[np.ndarray, float]],
    crossing_speed: float,
) -> Expr:
    """
    Generate the position-time equation for a computed crossing speed acceleration profile.

    Parameters:
        initial_speed: EGO vehicle starting speed.
        max_accel: EGO vehicle acceleration rate.
        d1: The distance traveled at the initial speed before acceleration begins.
        crossing_speed: EGO vehicle speed during crossing.

    Returns:
        A SymPy Piecewise expression defining the vehicle's position as a function of time (t).
    """
    t1 = d1 / initial_speed
    t2 = t1 + (crossing_speed - initial_speed) / max_accel
    pos1 = initial_speed * t
    pos2 = initial_speed * t1 + initial_speed*(t - t1) + 0.5 * max_accel*(t - t1)**2
    pos3 = initial_speed * t1 + initial_speed*(t2 - t1) + 0.5 * max_accel*(t2 - t1)**2 + crossing_speed * (t - t2)
    return Piecewise((pos1, t <= t1), (pos2, (t > t1) & (t <= t2)), (pos3, True))


def position_crossing_speed_decel(
    initial_speed: float,
    crossing_speed: Union[np.ndarray, float],
    decel: float,
    negotiation_offset: float,
) -> Expr:
    """
    Generate the position-time equation for a computed crossing speed deceleration profile.

    Parameters:
        initial_speed: EGO vehicle starting speed.
        crossing_speed: EGO vehicle speed during crossing.
        decel: EGO vehicle deceleration rate.
        negotiation_offset: Time required to cross the negotiation zone.

    Returns:
        A SymPy Piecewise expression defining the vehicle's position as a function of time (t).
    """
    t1 = negotiation_offset
    t2 = t1 + (initial_speed - crossing_speed) / decel
    pos1 = initial_speed * t
    pos2 = initial_speed * t1 + initial_speed * (t- t1) - 0.5 * decel * (t - t1)**2
    pos3 = initial_speed * t1 + initial_speed * (t2- t1) - 0.5 * decel * (t2 - t1)**2 + crossing_speed * (t - t2)
    return Piecewise((pos1, t <= t1), (pos2, (t > t1) & (t <= t2)), (pos3, True))


def position_midway_speed(
    initial_speed: float,
    crossing_speed: float,
    midway_speed: Union[np.ndarray, float],
    max_accel: float,
    decel: float,
    update_distance: Union[np.ndarray, float],
    negotiation_offset: float,
) -> Expr:
    """
    Generate the position-time equation for speed profiles A4 and B3.
    
    Parameters:
        initial_speed: EGO vehicle starting speed.
        crossing_speed: EGO vehicle speed during crossing.
        midway_speed: The intermediate low speed held to delay the vehicle.
        max_accel: EGO vehicle acceleration rate.
        decel: EGO vehicle deceleration rate.
        update_distance: Total distance covered by the initial deceleration and the midway cruise phase.
        negotiation_offset: Time required to cross the negotiation zone.

    Returns:
        A SymPy Piecewise expression defining the vehicle's position as a function of time (t).
    """
    t1 = negotiation_offset
    t2 = t1 + (initial_speed - midway_speed) / decel
    d2 = (initial_speed**2 - midway_speed**2) / (2 * decel)
    t3 = t1 + t2 + (update_distance - d2) / midway_speed
    t4 = t1 + t2 + t3 + (crossing_speed - midway_speed)/max_accel
    pos1 = initial_speed * t
    pos2 = initial_speed * t1 + initial_speed * (t - t1) - 0.5 * decel * (t - t1)**2
    pos3 = initial_speed * t1 + initial_speed * (t2 - t1) - 0.5 * decel * (t2 - t1)**2 + midway_speed * (t - t2)
    pos4 = (
        initial_speed * t1
        + initial_speed * (t2 - t1) - 0.5 * decel * (t2 - t1)**2
        + midway_speed*(t3 - t2)
        + midway_speed * (t - t3) + 0.5 * max_accel * (t - t3)**2
    )
    pos5 = (
        initial_speed * t1
        + initial_speed * (t2 - t1) - 0.5 * decel * (t2 - t1)**2
        + midway_speed*(t3 - t2)
        + midway_speed * (t4 - t3) + 0.5 * max_accel * (t4 - t3)**2
        + crossing_speed * (t - t4)
    )
    return Piecewise((pos1, t <= t1), (pos2, (t > t1) & (t <= t2)), (pos3, (t > t2) & (t <= t3)), (pos4, (t > t3) & (t <= t4)), (pos5, True))


def position_decel_distance(
    initial_speed: float,
    crossing_speed: float,
    decel: float,
    d1: Union[np.ndarray, float],
    negotiation_length: float,
) -> Expr:
    """
    Generate the position-time equation for a deceleration phase starting after a specific distance 'd1'.

    Parameters:
        initial_speed: EGO vehicle starting speed.
        crossing_speed: EGO vehicle speed during crossing.
        decel: EGO vehicle deceleration rate.
        d1: The distance traveled at the initial speed before deceleration begins.
        negotiation_length: Length of the negotiation zone.

    Returns:
        A SymPy Piecewise expression defining the vehicle's position as a function of time (t).
    """
    t1 = (d1 + negotiation_length) / initial_speed
    t2 = t1 + (initial_speed - crossing_speed) / decel
    pos1 = initial_speed * t
    pos2 = initial_speed * t1 + initial_speed * (t - t1) - 0.5 * decel * (t - t1)**2
    pos3 = initial_speed * t1 + initial_speed * (t2 - t1) - 0.5 * decel * (t2 - t1)**2 + crossing_speed * (t - t2)
    return Piecewise((pos1, t <= t1), (pos2, (t > t1) & (t <= t2)), (pos3, True))
