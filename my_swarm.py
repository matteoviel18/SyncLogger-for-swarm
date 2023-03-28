#! /usr/bin/env python3

# swarm

import cflib.crtp
from cflib.crazyflie import Commander
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import Swarm, _Factory
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

import actionlib
import rospy

from crazy_common_py.common_functions import standardNameList
from crazyflie_messages.msg import TakeoffAction, TakeoffResult, SwarmStates, CrazyflieState
from std_msgs.msg import Empty


# ++++++++++++++++++++++++ CREATE LIST OF URIS +++++++++++++++++++++++++++++++++

def create_uris_list():
    uris = []

    uri = 'radio://0/80/2M/E7E7E7E7E4'
    uris.append(uri)
    uri = 'radio://0/80/2M/E7E7E7E7E7'
    uris.append(uri)

    return uris


# ==================================================================================================================
#
#        L O G G E R S     C O N F I G U R A T I O N    M E T H O D S
#
# ==================================================================================================================

# Dictionaries are used to store the SyncLogger instances and extract data
# within the pace callback
# The key used to identify the drones is the URI string

# ++++++++++++++++++++++++ CREATE STATE LOGGERS +++++++++++++++++++++++++++++++++

def state_logger_drone(scf):
    # SyncLogger instantiation
    state_logger = SyncLogger(scf, state_logger_config)
    state_logger.connect()
    # Storing SyncLogger instance inside the dictionary
    state_loggers[scf._link_uri] = state_logger


def state_loggers_swarm():
    """
    Configuring the loggers to be used
    """
    swarm.parallel_safe(state_logger_drone)


# ==================================================================================================================
#
#         I N I T I A L  O P E R A T I O N S  M E T H O D S
#
# ==================================================================================================================

# ++++++++ MOTION COMMANDER AND COMMANDER INSTANTIATION METHOD +++++++++++++++++

def create_commanders_dict_drone(scf):
    # Motion commander instance
    motion_commander = MotionCommander(scf)
    mc_dict[scf._link_uri] = motion_commander


def create_commanders_dict_swarm():
    swarm.parallel_safe(create_commanders_dict_drone)


# ++++++++ STATE SUBSCRIBER LIST METHOD +++++++++++++++++++

def make_states_list():
    for cf_name in cf_names:
        states.append(CrazyflieState())


# ------------------------------------------------------------------------------------------------------------------
#
#                                  P A C E _ S U B _ C A L L B A C K
#
# This callback is called whenever an Empty message is published by pace_state node
# ------------------------------------------------------------------------------------------------------------------
def pace_sub_callback(msg):
    if initialOperationsEnded:

        # Initializing states message:
        states_ = SwarmStates()

        cf_index = 0

        for uri in uris:
            # Extracting information from the loggers
            state_data_dict[uri] = state_loggers[uri].next()

            print(uri, state_data_dict[uri])  # debug
            # ERROR: different drones have the same states

            # States
            states[cf_index].name = 'cf' + str(cf_index + 1)
            states[cf_index].position.x = \
                state_data_dict[uri][1]['stateEstimate.x']
            states[cf_index].position.y = \
                state_data_dict[uri][1]['stateEstimate.y']
            states[cf_index].position.z = \
                state_data_dict[uri][1]['stateEstimate.z']
            states[cf_index].velocity.x = \
                state_data_dict[uri][1]['stateEstimate.vx']
            states[cf_index].velocity.y = \
                state_data_dict[uri][1]['stateEstimate.vy']
            states[cf_index].velocity.z = \
                state_data_dict[uri][1]['stateEstimate.vz']

            cf_index += 1

        states_.states = states

        states_pub.publish(states_)  # publish swarm state


# ==================================================================================================================
#
#                               T A K E    O F F
#
# ==================================================================================================================

def swarm_takeoff():

    print('take off...')

    takeoff_swarm()

    rospy.sleep(2)


# +++++++++++++++++++++++ TAKEOFF METHOD ++++++++++++++++++++++++++++++++++++++

# for any drone of the swarm
def takeoff_drone4swarm(scf):
    mc_dict[scf._link_uri].take_off(height=0.3)  # <- modifica altezza decollo /a discrezione dell'utente


def takeoff_swarm():
    print('takeoff action')
    swarm.parallel_safe(takeoff_drone4swarm)


# # ==================================================================================================================
#
#                                    L A N D
#
# # ==================================================================================================================
def swarm_land():

    print('landing...')

    land_swarm()

    rospy.sleep(2)


# +++++++++++++++++++++++++ LAND METHOD +++++++++++++++++++++++++++++++++++++++

# for any drone of the swarm
def land_drone4swarm(scf):
    mc_dict[scf._link_uri].land()


def land_swarm():
    print('land action')
    swarm.parallel_safe(land_drone4swarm)


##################################################################

#                            M A I N

##################################################################

if __name__ == '__main__':
    # Node initialization
    rospy.init_node('node_swarm', log_level=rospy.DEBUG)

    initialOperationsEnded = False

    # Number of crazyflies
    N_cf = 2

    # Generate a standard list of names
    cf_names = standardNameList(N_cf)

    uris = create_uris_list()

    # Dictionaries for commanders initialization
    mc_dict = dict()

    # Logger period
    logger_period = 200  # ms

    # State logger configuration (6 floats => 24/26 bytes)
    state_logger_config = LogConfig(name='state_conf',
                                    period_in_ms=logger_period)
    state_logger_config.add_variable('stateEstimate.x',
                                     'float')
    state_logger_config.add_variable('stateEstimate.y',
                                     'float')
    state_logger_config.add_variable('stateEstimate.z',
                                     'float')
    state_logger_config.add_variable('stateEstimate.vx',
                                     'float')
    state_logger_config.add_variable('stateEstimate.vy',
                                     'float')
    state_logger_config.add_variable('stateEstimate.vz',
                                     'float')

    # Dictionaries for logger data
    state_data_dict = dict()

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 S U B S C R I B E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # Subscriber to pace:
    pace_sub = rospy.Subscriber('/pace_state', Empty, pace_sub_callback, queue_size=1)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #                 P U B L I S H E R S  S E T U P
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # States publisher:
    states_pub = rospy.Publisher('/swarm/states', SwarmStates, queue_size=1)

    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #             I N I T I A L  O P E R A T I O N S
    # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    # print('Loggers ok')

    # Drivers initialization:
    cflib.crtp.init_drivers()

    # Instantiation of swarm:
    swarm = Swarm(uris, factory=_Factory())

    #  Opening communication
    swarm.open_links()

    # Resetting estimators
    swarm.reset_estimators()

    # Loggers configuration for all the agents
    state_loggers = dict()

    # Configuring the loggers
    state_loggers_swarm()

    # List of states as CrazyflieState messages:
    states = []
    make_states_list()

    # Create commanders for the swarm
    create_commanders_dict_swarm()

    # swarm_takeoff()

    initialOperationsEnded = True

    while not rospy.is_shutdown():
        pass

    # swarm_land()

    rospy.spin()
