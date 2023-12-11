#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Collection of traffic scenarios where the ego vehicle (hero)
is making a left turn
"""

from six.moves.queue import Queue  # pylint: disable=relative-import,bad-option-value

import py_trees
import carla
from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      ActorSource,
                                                                      ActorSink,
                                                                      Idle,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import generate_target_waypoint


class LeftTurnBetweenTwoCars(BasicScenario):

    """
    Implementation class for Hero
    Vehicle turning left at signalized junction scenario,
    Traffic Scenario 08.

    This is a single ego vehicle scenario
    """

    timeout = 80  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=80):
        """
        Setup all relevant parameters and create scenario
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._target_vel = 6.9
        self._brake_value = 0.5
        self._ego_distance = 110
        self._traffic_light = None
        self._other_actor_transform = None
        self._blackboard_queue_name = 'SignalizedJunctionLeftTurn/actor_flow_queue'
        self._queue = py_trees.blackboard.Blackboard().set(self._blackboard_queue_name, Queue())
        self._initialized = True
        super(LeftTurnBetweenTwoCars, self).__init__("LeftTurnBetweenTwoCars",
                                                         ego_vehicles,
                                                         config,
                                                         world,
                                                         debug_mode,
                                                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[0].model, self._other_actor_transform)
        first_vehicle.set_transform(first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)
        
        #=============added===========================
        #initialize second actor
        self._other_actor_transform_2 = config.other_actors[1].transform
        second_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[1].transform.location.x,
                           config.other_actors[1].transform.location.y,
                           config.other_actors[1].transform.location.z - 500),
            config.other_actors[1].transform.rotation)
        second_vehicle = CarlaDataProvider.request_new_actor(config.other_actors[1].model, self._other_actor_transform_2)
        second_vehicle.set_transform(second_vehicle_transform)
        second_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(second_vehicle)
        #===============================================

    def _create_behavior(self):
        """
        Hero vehicle is turning left in an urban area,
        at a signalized intersection, while other actor coming straight
        .The hero actor may turn left either before other actor
        passes intersection or later, without any collision.
        After 80 seconds, a timeout stops the scenario.
        """

        sequence = py_trees.composites.Sequence("Sequence Behavior")

        # Selecting straight path at intersection
        target_waypoint = generate_target_waypoint(
            CarlaDataProvider.get_map().get_waypoint(self.other_actors[0].get_location()), 0)
       
        drive_car1 = WaypointFollower(self.other_actors[0], 30,
                                      blackboard_queue_name=self._blackboard_queue_name, avoid_collision=True)
        
        drive_car2 = WaypointFollower(self.other_actors[1], 5,
                                      blackboard_queue_name=self._blackboard_queue_name, avoid_collision=True)
        wait = Idle(0)
        
        car2_sequence = py_trees.composites.Sequence("car2 behavior")
        car2_sequence.add_child(wait)
        car2_sequence.add_child(drive_car2)
        # Behavior tree
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(drive_car1)
        root.add_child(car2_sequence)
        
        
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        sequence.add_child(ActorTransformSetter(self.other_actors[1], self._other_actor_transform_2))
        sequence.add_child(root)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicles[0])
        criteria.append(collison_criteria)

        return criteria

    def __del__(self):
        self._traffic_light = None
        self.remove_all_actors()
