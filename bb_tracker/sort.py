
"""
    SORT: A Simple, Online and Realtime Tracker
    Copyright (C) 2016-2020 Alex Bewley alex@bewley.ai

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import print_function
from filterpy.kalman import KalmanFilter
import numpy as np


def linear_assignment(cost_matrix):
    try:
        import lap
        _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
        return np.array([[y[i], i] for i in x if i >= 0])
    except ImportError:
        from scipy.optimize import linear_sum_assignment
        x, y = linear_sum_assignment(cost_matrix)
        return np.array(list(zip(x, y)))


def iou_batch(bb_test, bb_gt):
    """
    From SORT: Computes Intersection-over-Union (IoU) between two bboxes in the form [x1,y1,x2,y2]
    """
    bb_gt = np.expand_dims(bb_gt, 0)
    bb_test = np.expand_dims(bb_test, 1)

    xx1 = np.maximum(bb_test[..., 0], bb_gt[..., 0])
    yy1 = np.maximum(bb_test[..., 1], bb_gt[..., 1])
    xx2 = np.minimum(bb_test[..., 2], bb_gt[..., 2])
    yy2 = np.minimum(bb_test[..., 3], bb_gt[..., 3])
    w = np.maximum(0., xx2 - xx1)
    h = np.maximum(0., yy2 - yy1)
    wh = w * h
    o = wh / ((bb_test[..., 2] - bb_test[..., 0]) * (bb_test[..., 3] - bb_test[..., 1])
              + (bb_gt[..., 2] - bb_gt[..., 0]) * (bb_gt[..., 3] - bb_gt[..., 1]) - wh)
    return (o)


def convert_bbox_to_z(bbox):
    """
    Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
      [x,y,s,r] where x,y is the centre of the box and s is the scale/area and r is
      the aspect ratio
    """
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]
    x = bbox[0] + w/2.
    y = bbox[1] + h/2.
    s = w * h  # scale is the area of bbox
    r = w / float(h)
    return np.array([x, y, s, r]).reshape((4, 1))


def convert_z_to_bbox(x):
    """
    Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
      [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
    """
    w = np.sqrt(x[2] * x[3])
    h = x[2] / w

    return np.array([x[0]-w/2., x[1]-h/2., x[0]+w/2., x[1]+h/2.]).reshape((1, 4))


class KalmanBoxTracker(object):
    """
    This class represents the internal state of individual tracked objects observed as bbox.
    """
    count = 0       # used to give IDs to objects

    def __init__(self, bbox, dt=1.0):
        """
        Initialises a tracker using an initial bounding box.
        """
        # define constant velocity model
        # aspect ratio of object, width and height are assumed to be constant
        # x = [center_x, center_y, scale, ratio, x_dot, y_dot]
        # y = [center_x, center_y, scale, ratio]

        self.kf = KalmanFilter(dim_x=6, dim_z=4)
        self.kf.F = np.array([[1, 0, 0, 0, dt, 0],
                              [0, 1, 0, 0, 0, dt],
                              [0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0]])

        # measurement noise: (dim_z, dim_z)
        # assuming all variables are independent
        std_meas = 1.0
        self.kf.R = np.array([[std_meas**2, 0, 0, 0],
                              [0, std_meas**2, 0, 0],
                              [0, 0, 0.1**2, 0],
                              [0, 0, 0, 0.1**2]])

        # covariance matrix: (dim_x, dim_x), initial
        # give high uncertainty to the unobservable initial velocities
        self.kf.P = np.array([[0.5, 0, 0, 0, 0, 0],
                              [0, 0.5, 0, 0, 0, 0],
                              [0, 0, 0.5, 0, 0, 0],
                              [0, 0, 0, 0.5, 0, 0],
                              [0, 0, 0, 0, 1000., 0],
                              [0, 0, 0, 0, 0, 1000.]])

        # process noise: (dim_x, dim_x)
        std_pos = 0.05
        std_vel = 0.05
        self.kf.Q = np.array([[std_pos**2, 0, 0, 0, std_pos*std_vel, 0],
                              [0, std_pos**2, 0, 0, 0, std_pos*std_vel],
                              [0, 0,          0.1**2, 0, 0, 0],
                              [0, 0,          0, 0.1**2, 0, 0],
                              [std_pos*std_vel, 0, 0, 0, std_vel**2, 0],
                              [0, std_pos*std_vel, 0, 0, 0, std_vel**2]])

        self.kf.x[:4] = convert_bbox_to_z(bbox)
        self.time_since_update = 0
        self.x_history = []           # list of past states in [x,y,s,r]
        self.k_history = []           # list of past Kalman gain values used in update
        self.hits = 0
        self.hit_streak = 0
        self.age = 0
        self.id = KalmanBoxTracker.count    # object id
        KalmanBoxTracker.count += 1

    def update(self, bbox):
        """
        Updates the state vector with observed bbox.
        """
        self.time_since_update = 0
        self.x_history = []
        self.hits += 1
        self.hit_streak += 1
        self.kf.update(convert_bbox_to_z(bbox))
        self.k_history.append(self.kf.K)

    def predict(self):
        """
        Advances the state vector and returns the predicted bounding box estimate.
        """
        self.kf.predict()
        self.age += 1
        if (self.time_since_update > 0):
            self.hit_streak = 0
        self.time_since_update += 1
        self.x_history.append(convert_z_to_bbox(self.kf.x))
        return self.x_history[-1], self.id

    def get_state(self):
        """
        Returns the current bounding box estimate.
        """
        return convert_z_to_bbox(self.kf.x)


def associate_detections_to_trackers(detections, trackers, iou_threshold=0.3):
    """
    Assigns detections to tracked object (both represented as bounding boxes)

    Returns
    --------
        matches : np.array 
            Contains the matched pairs with 2 columns of id of detection and id of tracker

        unmatched_detections : np.array
            Array of indices of detections without a matching tracking

        unmatched_trackers : np.array
            Array of indices of trackers without a matching detection
    """
    if (len(trackers) == 0):
        return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0, 5), dtype=int)

    # Compute IoU scores bewteen detections and trackers
    iou_matrix = iou_batch(detections, trackers)

    # Find matches
    if min(iou_matrix.shape) > 0:
        a = (iou_matrix > iou_threshold).astype(np.int32)
        if a.sum(1).max() == 1 and a.sum(0).max() == 1:
            matched_indices = np.stack(np.where(a), axis=1)
        else:
            matched_indices = linear_assignment(-iou_matrix)
    else:
        matched_indices = np.empty(shape=(0, 2))

    # Find indices of unmatched detections
    unmatched_detections = []
    for d, det in enumerate(detections):
        if (d not in matched_indices[:, 0]):
            unmatched_detections.append(d)

    # Find indices of unmatched trackers
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if (t not in matched_indices[:, 1]):
            unmatched_trackers.append(t)

    # Filter out matches with low IOU
    matches = []
    for m in matched_indices:
        if (iou_matrix[m[0], m[1]] < iou_threshold):
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))

    if (len(matches) == 0):
        matches = np.empty((0, 2), dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)

    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


class Sort(object):
    def __init__(self, max_age=15, min_hits=2, iou_threshold=0.2, dt=0.2, max_speed=2.0):
        """
        Sets key parameters for SORT
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.max_speed = max_speed
        self.iou_threshold = iou_threshold
        self.trackers: list[KalmanBoxTracker] = []
        self.frame_count = 0
        self.dt = dt

    def update(self, dets=np.empty((0, 5))):
        """
        Parameters
        -------
        dets : np.array 
            Detections in the format [[x1,y1,x2,y2],[x1,y1,x2,y2],...]

        Returns
        -------
        detected_bbs : np.array
            BBs of objects that are tracked and detected, with each row as [x1,y1,x2,y2,tracker_id]

        undetected_bbs : np.array
            BBs of objects that are tracked but undetected, with each row as [x1,y1,x2,y2,tracker_id]
        """

        self.frame_count += 1
        # Get predicted locations from existing trackers.
        trks = np.zeros((len(self.trackers), 5))
        to_del = []
        detected_bbs = []
        for t, trk in enumerate(trks):
            pos, id_ = self.trackers[t].predict()
            # pos = [[x,y,s,r]]
            pos = pos[0]
            trk[:] = [pos[0], pos[1], pos[2], pos[3], id_]
            if np.any(np.isnan(pos)):
                to_del.append(t)
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))

        # Delete trackers with nan position values
        for t in reversed(to_del):
            self.trackers.pop(t)

        # Associate detections to trackers
        matched_ids, unmatched_dets, unmatched_trackers = associate_detections_to_trackers(
            dets, trks, self.iou_threshold)

        # Update matched trackers with assigned detections
        for m in matched_ids:
            # Update step in Kalman filter
            self.trackers[m[1]].update(dets[m[0], :])

        # Get bboxes of undetected_ids
        undetected_bbs = []
        for idx in unmatched_trackers:
            bbox = trks[idx].tolist()
            undetected_bbs.append(bbox)

        # Initialise new trackers for unmatched detections
        for idx in unmatched_dets:
            trk = KalmanBoxTracker(dets[idx, :], dt=self.dt)
            self.trackers.append(trk)

        # Return valid trackers
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            d = trk.get_state()[0]
            if (trk.time_since_update < 1) and \
                    (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
                detected_bbs.append(np.concatenate(
                    (d, [trk.id])).reshape(1, -1))
            i -= 1
            # Remove dead tracklet
            if (trk.time_since_update > self.max_age):
                self.trackers.pop(i)
            # Remove due to improbably high speed
            if abs(trk.kf.x[4]) > self.max_speed or abs(trk.kf.x[5]) > self.max_speed:
                self.trackers.pop(i)

        if (len(detected_bbs) > 0):
            return np.concatenate(detected_bbs), undetected_bbs


        return np.empty((0, 5)), undetected_bbs

    def update_without_detections(self):
        '''
        Predict existing trackers when there are no detections.

        Returns
        -------
        undetected_bbs : list
            BBs of objects that are tracked but undetected, with each row as [x1,y1,x2,y2,track_id]
        '''
        self.frame_count += 1
        trks = np.zeros((len(self.trackers), 5))
        to_del = []
        undetected_bbs = []

        # Get predicted locations from existing trackers.
        for t, trk in enumerate(trks):
            pos, id_ = self.trackers[t].predict()
            # pos = [[x,y,s,r]]
            pos = pos[0]
            trk[:] = [pos[0], pos[1], pos[2], pos[3], id_]
            if np.any(np.isnan(pos)):
                to_del.append(t)
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))

        # Delete trackers with nan position values
        for t in reversed(to_del):
            self.trackers.pop(t)

        # Return valid trackers
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            d = trk.get_state()[0]      # [x1,y1,x2,y2]
            bbox = np.concatenate((d, [trk.id])).reshape(1, -1)
            undetected_bbs.append(bbox)
            i -= 1
            # Remove dead tracklet
            if (trk.time_since_update > self.max_age):
                self.trackers.pop(i)
            # Remove due to improbably high speed
            if abs(trk.kf.x[4]) > self.max_speed or abs(trk.kf.x[5]) > self.max_speed:
                self.trackers.pop(i)

        if (len(undetected_bbs) > 0):
            return np.concatenate(undetected_bbs)


        return np.empty((0, 5))
    