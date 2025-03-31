"""We define the Simulator class which stores parameters so that certain common
operations, like inflating the occupancy grid, computing simulated sensor
measurements, and plotting are easily computed.
"""
import logging
import math
import time

from . import laser, mapping, utils, core
from .constants import UNOBSERVED_VAL, FREE_VAL


class Simulator(object):
    def __init__(self,
                 args,
                 verbose=True):
        """args requires a number of values:

        - base_resolution (float) resolution of a single grid cell
        - inflation_radius_m (float) inflation radius of grid (in meters)
        - laser_max_range_m (float) max range of laser scanner (in meters)
        - field_of_view_deg (float) laser scanner field of view
        - laser_scanner_num_points (int) number of points in the sim scan
        - current_seed (int) seed for map generation (only used to name logs)
        """
        # Store some necesasry data and arguments
        self.args = args
        self.resolution = args.base_resolution
        self.inflation_radius = args.inflation_radius_m / self.resolution
        self.frontier_grouping_inflation_radius = 0

        self.laser_max_range_m = args.laser_max_range_m
        self.disable_known_grid_correction = args.disable_known_grid_correction

        # Create the directions object
        self.laser_scanner_num_points = args.laser_scanner_num_points
        self.directions = laser.get_laser_scanner_directions(
            num_points=self.laser_scanner_num_points,
            field_of_view_rad=math.radians(args.field_of_view_deg))

        self.verbose = verbose

    def set_known_map(self, known_map):
        known_map = known_map.copy()
        known_map[known_map == -1] = FREE_VAL

        self.known_map = known_map
        self.inflated_known_grid = utils.inflate_grid(
            known_map, inflation_radius=self.inflation_radius)

    def get_laser_scan(self, robot_pose):
        """Get a simulated laser scan."""
        # Get the laser scan
        ranges = laser.simulate_sensor_measurement(
            self.known_map,
            self.directions,
            max_range=self.laser_max_range_m / self.resolution + 2,
            sensor_pose=robot_pose)

        return ranges

    def get_laser_scan_and_update_map(self,
                                      robot_pose,
                                      observed_map,
                                      get_newly_observed=False):
        """Get the simulate laser scan and insert it into the grid."""
        logger = logging.getLogger("simulators")
        stime = time.time()
        ranges = self.get_laser_scan(robot_pose)
        directions = self.directions[:, ranges < self.laser_max_range_m / self.resolution]
        ranges = ranges[ranges < self.laser_max_range_m / self.resolution]
        logger.debug(f"time to get laser scan: {time.time() - stime}")

        if not self.disable_known_grid_correction:
            return self._update_map_with_correction(robot_pose, directions,
                                                    ranges,
                                                    observed_map,
                                                    get_newly_observed)

        # Insert the scan
        stime = time.time()
        observed_map = mapping.insert_scan(observed_map,
                                           directions,
                                           laser_ranges=ranges,
                                           max_range=self.laser_max_range_m /
                                           self.resolution,
                                           sensor_pose=robot_pose,
                                           connect_neighbor_distance=2)
        logger.debug(f"time to insert laser scan: {time.time() - stime}")

        # Optionally get and return the visibility mask
        if get_newly_observed:
            newly_observed_grid = mapping.insert_scan(
                0 * observed_map - 1,
                directions,
                laser_ranges=ranges,
                max_range=self.laser_max_range_m / self.resolution,
                sensor_pose=robot_pose,
                connect_neighbor_distance=2)

        # Optionally "correct" the grid using the known map. This compensates
        # for errors in the reprojection of the laser scan introduces by the
        # Bresenham line algorithm used for ray tracing.
        if not self.disable_known_grid_correction:
            known = self.known_map.copy()
            mask = (observed_map == UNOBSERVED_VAL)
            known[mask] = UNOBSERVED_VAL
            observed_map = known

            if get_newly_observed:
                known = self.known_map.copy()
                mask = (newly_observed_grid == UNOBSERVED_VAL)
                known[mask] = UNOBSERVED_VAL
                newly_observed_grid = known

        if get_newly_observed:
            return ranges, observed_map, newly_observed_grid
        else:
            return ranges, observed_map

    def _update_map_with_correction(self, robot_pose, directions, ranges, observed_map,
                                    get_newly_observed):
        newly_observed_grid = mapping.insert_scan(
            observed_map,
            directions,
            laser_ranges=ranges,
            max_range=self.laser_max_range_m / self.resolution,
            sensor_pose=robot_pose,
            do_only_compute_visibility=True)

        new_visibility_mask = (newly_observed_grid != UNOBSERVED_VAL)
        observed_map[new_visibility_mask] = self.known_map[new_visibility_mask]

        if get_newly_observed:
            newly_observed_grid[new_visibility_mask] = self.known_map[
                new_visibility_mask]
            return ranges, observed_map, newly_observed_grid
        else:
            return ranges, observed_map

    def get_updated_frontier_set(self, inflated_grid, robot_pose, saved_frontiers):
        """Compute the frontiers, store the new ones and compute properties."""
        new_frontiers = core.get_frontiers(
            inflated_grid,
            group_inflation_radius=self.frontier_grouping_inflation_radius)
        saved_frontiers = core.update_frontier_set(saved_frontiers,
                                                       new_frontiers)

        return saved_frontiers

    def get_inflated_grid(self, observed_map, robot_pose):
        """Compute the inflated grid."""
        # Inflate the grid and generate a plan
        inflated_grid = utils.inflate_grid(
            observed_map, inflation_radius=self.inflation_radius)

        # Prevents robot from getting stuck occasionally: sometimes (very
        # rarely) the robot would reveal an obstacle and then find itself
        # within the inflation radius of that obstacle. This should have
        # no side-effects, since the robot is expected to be in free space.
        inflated_grid[int(robot_pose.x), int(robot_pose.y)] = 0

        return inflated_grid
