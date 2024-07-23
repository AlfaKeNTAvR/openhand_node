#!/usr/bin/env python
"""

Author(s):

TODO:

"""

# # Standart libraries:
import rospy
import numpy as np

# # Third party libraries:
import hands

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    Float32,
)

# # Third party messages and services:


class OpenHandModelO:
    """
    
    """

    def __init__(
        self,
        node_name,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name

        self.__MAX_INDEX_FINGER = 0.7
        self.__MAX_MIDDLE_FINGER = 0.7
        self.__MAX_THUMB_FINGER = 0.7

        self.__T = hands.Model_O(
            "/dev/ttyUSB0",
            1,
            4,
            3,
            2,
            "XM",
            0.0,
            0.08,
            -0.05,
            0.23,
        )

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__adduct = 0.0
        self.__index_finger = 0.0
        self.__middle_finger = 0.0
        self.__thumb_finger = 0.0

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.__NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {
            # 'dependency_node_name': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        # self.__dependency_status_topics['<dependency_node_name>'] = (
        #     rospy.Subscriber(
        #         f'/<dependency_node_name>/is_initialized',
        #         Bool,
        #         self.__dependency_name_callback,
        #     )
        # )

        # # Service provider:
        # rospy.Service(
        #     f'{self.__NODE_NAME}/<service_name1>',
        #     SetBool,
        #     self.__service_name1_handler,
        # )

        # # Service subscriber:
        # self.__service = rospy.ServiceProxy(
        #     '/<service_name2>',
        #     ServiceType2,
        # )

        # # Topic publisher:
        # self.__publisher = rospy.Publisher(
        #     f'{self.__NODE_NAME}/<topic_name1>',
        #     Bool,
        #     queue_size=1,
        # )

        # # Topic subscriber:
        rospy.Subscriber(
            f'{self.__NODE_NAME}/index_finger',
            Float32,
            self.__index_finger_callback,
        )
        rospy.Subscriber(
            f'{self.__NODE_NAME}/middle_finger',
            Float32,
            self.__middle_finger_callback,
        )
        rospy.Subscriber(
            f'{self.__NODE_NAME}/thumb_finger',
            Float32,
            self.__thumb_finger_callback,
        )

        # # Timers:
        rospy.Timer(
            rospy.Duration(1.0 / 100),
            self.__send_motor_commands,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    # def __dependency_name_callback(self, message):
    #     """Monitors <node_name>/is_initialized topic.

    #     """

    #     # self.__dependency_status['dependency_node_name'] = message.data

    # # Service handlers:
    # def __service_name1_handler(self, request):
    #     """

    #     """

    #     response = True

    #     return response

    # # Topic callbacks:
    def __index_finger_callback(self, message):
        """

        """

        self.__index_finger = np.clip(
            message.data,
            0.0,
            self.__MAX_INDEX_FINGER,
        )

    def __middle_finger_callback(self, message):
        """

        """

        self.__middle_finger = np.clip(
            message.data,
            0.0,
            self.__MAX_MIDDLE_FINGER,
        )

    def __thumb_finger_callback(self, message):
        """

        """

        self.__thumb_finger = np.clip(
            message.data,
            0.0,
            self.__MAX_THUMB_FINGER,
        )

    # # Timer callbacks:
    def __send_motor_commands(self, event):
        """Calls <some_function> on each timer callback with 100 Hz frequency.
        
        """

        self.__T.moveMotor(0, self.__adduct)
        self.__T.moveMotor(1, self.__index_finger)
        self.__T.moveMotor(2, self.__middle_finger)
        self.__T.moveMotor(3, self.__thumb_finger)

    # # Private methods:
    # NOTE: By default all new class methods should be private.
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes' is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (f'{self.__NODE_NAME}: '
                         f'lost connection to {key}!')
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.__NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    # # Public methods:
    # NOTE: By default all new class methods should be private.
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'openhand_model_o',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    class_instance = OpenHandModelO(node_name=node_name,)

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
