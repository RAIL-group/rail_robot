import numpy as np
import math
import scipy.ndimage

from .constants import (FREE_VAL, UNOBSERVED_VAL,
                               OBSTACLE_THRESHOLD)

from . import utils

IS_FROM_LAST_CHOSEN_REWARD = 0 * 10.0


class Frontier(object):
    def __init__(self, points):
        """Initialized with a 2xN numpy array of points (the grid cell
        coordinates of all points on frontier boundary)."""
        inds = np.lexsort((points[0, :], points[1, :]))
        sorted_points = points[:, inds]
        self.props_set = False
        self.is_from_last_chosen = False
        self.is_obstructed = False
        self.prob_feasible = 1.0
        self.delta_success_cost = 0.0
        self.exploration_cost = 0.0
        self.negative_weighting = 0.0
        self.positive_weighting = 0.0

        self.counter = 0
        self.last_observed_pose = None

        # Any duplicate points should be eliminated (would interfere with
        # equality checking).
        dupes = []
        for ii in range(1, sorted_points.shape[1]):
            if (sorted_points[:, ii - 1] == sorted_points[:, ii]).all():
                dupes += [ii]
        self.points = np.delete(sorted_points, dupes, axis=1)

        # Compute and cache the hash
        self.hash = hash(self.points.tobytes())

    def set_props(self,
                  prob_feasible,
                  is_obstructed=False,
                  delta_success_cost=0,
                  exploration_cost=0,
                  positive_weighting=0,
                  negative_weighting=0,
                  counter=0,
                  last_observed_pose=None,
                  did_set=True):
        self.props_set = did_set
        self.just_set = did_set
        self.prob_feasible = prob_feasible
        self.is_obstructed = is_obstructed
        self.delta_success_cost = delta_success_cost
        self.exploration_cost = exploration_cost
        self.positive_weighting = positive_weighting
        self.negative_weighting = negative_weighting
        self.counter = counter
        self.last_observed_pose = last_observed_pose

    @property
    def centroid(self):
        return self.get_centroid()

    def get_centroid(self):
        """Returns the point that is the centroid of the frontier"""
        centroid = np.mean(self.points, axis=1)
        return centroid

    def get_frontier_point(self):
        """Returns the point that is on the frontier that is closest to the
        actual centroid"""
        center_point = np.mean(self.points, axis=1)
        norm = np.linalg.norm(self.points - center_point[:, None], axis=0)
        ind = np.argmin(norm)
        return self.points[:, ind]

    def get_distance_to_point(self, point):
        norm = np.linalg.norm(self.points - point[:, None], axis=0)
        return norm.min()

    def __hash__(self):
        return self.hash

    def __eq__(self, other):
        return hash(self) == hash(other)


def get_frontiers(occupancy_grid, group_inflation_radius=0):
    """Get froniers from the map.

    Frontiers exist at the boundary between free and unknown space. The
    points that make up the frontiers exist in the *unknown* portion of
    the space. This helps avoid some planning issues later on; if frontiers
    are generated in the *known* portion of the map, they may obstruct
    the robot's path and erroneously rule out regions of the map.

    We compute the frontiers using connected components. Masked by all the
    frontiers, a map should confine an agent to the observed region.
    """
    filtered_grid = scipy.ndimage.maximum_filter(np.logical_and(
        occupancy_grid < OBSTACLE_THRESHOLD, occupancy_grid == FREE_VAL), size=3)
    frontier_point_mask = np.logical_and(filtered_grid,
                                         occupancy_grid == UNOBSERVED_VAL)

    if group_inflation_radius < 1:
        inflated_frontier_mask = frontier_point_mask
    else:
        inflated_frontier_mask = utils.inflate_grid(
            frontier_point_mask,
            inflation_radius=group_inflation_radius,
            obstacle_threshold=0.5,
            collision_val=1.0) > 0.5

    # Group the frontier points into connected components
    labels, nb = scipy.ndimage.label(inflated_frontier_mask)

    # Extract the frontiers
    frontiers = set()
    for ii in range(nb):
        raw_frontier_indices = np.where(
            np.logical_and(labels == (ii + 1), frontier_point_mask))
        frontiers.add(
            Frontier(
                np.concatenate((raw_frontier_indices[0][None, :],
                                raw_frontier_indices[1][None, :]),
                               axis=0)))

    return frontiers


def update_frontier_set(old_set, new_set, max_dist=None, chosen_frontier=None):
    """Updates an old set of frontiers with a new set of frontiers.

    If a frontier persists, it is kept. If a new frontier appears, it is added.
    Everything is done with python set operations. Finally, if a
    "chosen_frontier" is passed, any frontier that derives its properties (i.e.
    is closest to) that frontier from the old set has its 'is_from_last_chosen'
    property set to true.
    """

    # Update the 'just_set' and 'is_from_last_chosen' properties
    for frontier in old_set:
        frontier.just_set = False
        frontier.is_from_last_chosen = False

    # Shallow copy of the set
    old_set = old_set.copy()

    # These are the frontiers that will not appear in the new set
    outgoing_frontier_set = old_set - new_set
    added_frontier_set = new_set - old_set
    if max_dist is not None:
        # Loop through the newly added_frontier_set and set properties based
        # upon the outgoing_frontier_set
        for af in added_frontier_set:
            nearest_frontier, nearest_frontier_dist = (
                _get_nearest_feasible_frontier(
                    frontier=af,
                    reference_frontier_set=outgoing_frontier_set,
                ))
            if nearest_frontier_dist < max_dist:
                af.set_props(
                    prob_feasible=nearest_frontier.prob_feasible,
                    delta_success_cost=nearest_frontier.delta_success_cost,
                    exploration_cost=nearest_frontier.exploration_cost,
                    did_set=False)
                try:
                    af.image = nearest_frontier.image
                    af.mask = nearest_frontier.mask
                    af.goal_loc_x_mat = nearest_frontier.goal_loc_x_mat
                    af.goal_loc_y_mat = nearest_frontier.goal_loc_y_mat
                except AttributeError:
                    pass

                if nearest_frontier == chosen_frontier:
                    af.is_from_last_chosen = True
            else:
                af.set_props(prob_feasible=1.0,
                             delta_success_cost=0.0,
                             exploration_cost=0.0,
                             did_set=False)

    # Remove frontier_set that don't appear in the new set
    old_set.difference_update(outgoing_frontier_set)

    # Add the new frontier_set
    old_set.update(added_frontier_set)

    return old_set


def _eucl_dist(p1, p2):
    """Helper to compute Euclidean distance."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def _get_nearest_feasible_frontier(frontier, reference_frontier_set):
    """Returns the nearest 'feasible' frontier from a reference set."""
    f_gen = [(of, _eucl_dist(of.get_centroid(), frontier.get_centroid()))
             for of in reference_frontier_set if of.prob_feasible > 0.0]
    if len(f_gen) == 0:
        return None, 1e10
    else:
        return min(f_gen, key=lambda fd: fd[1])
