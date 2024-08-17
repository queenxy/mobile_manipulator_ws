# vim: expandtab:ts=4:sw=4
from __future__ import absolute_import
from datetime import datetime
import numpy as np
from . import kalman_filter
from . import linear_assignment
from . import iou_matching
from .track import Track


class Tracker:
    """
    This is the multi-target tracker.

    Parameters
    ----------
    metric : nn_matching.NearestNeighborDistanceMetric
        A distance metric for measurement-to-track association.
    max_age : int
        Maximum number of missed misses before a track is deleted.
    n_init : int
        Number of consecutive detections before the track is confirmed. The
        track state is set to `Deleted` if a miss occurs within the first
        `n_init` frames.
    today: Optional[datetime.date]
            Provide today's date, for naming of tracks

    Attributes
    ----------
    metric : nn_matching.NearestNeighborDistanceMetric
        The distance metric used for measurement to track association.
    max_age : int
        Maximum number of missed misses before a track is deleted.
    n_init : int
        Number of frames that a track remains in initialization phase.
    kf : kalman_filter.KalmanFilter
        A Kalman filter to filter target trajectories in image space.
    tracks : List[Track]
        The list of active tracks at the current time step.
    gating_only_position : Optional[bool]
        Used during gating, comparing KF predicted and measured states. If True, only the x, y position of the state distribution is considered during gating. Defaults to False, where x,y, aspect ratio and height will be considered.
    """

    def __init__(
        self,
        metric,
        max_iou_distance=0.7,
        max_age=30,
        n_init=3,
        override_track_class=None,
        today=None,
        gating_only_position=False
    ):
        self.today = today
        self.metric = metric
        self.max_iou_distance = max_iou_distance
        self.max_age = max_age
        self.n_init = n_init
        self.gating_only_position = gating_only_position

        self.kf = kalman_filter.KalmanFilter()
        self.tracks = []
        self.del_tracks_ids = []
        self._next_id = 1
        if override_track_class:
            self.track_class = override_track_class
        else:
            self.track_class = Track

    def predict(self):
        """Propagate track state distributions one time step forward.

        This function should be called once every time step, before `update`.
        """
        for track in self.tracks:
            track.predict(self.kf)

    def update(self, detections, today=None, frame=None, register_person=False) -> bool:
        """Perform measurement update and track management.

        Parameters
        ----------
        detections : List[deep_sort.detection.Detection]
            A list of detections at the current time step.
        today: Optional[datetime.date]
            Provide today's date, for naming of tracks
        """
        ret = 0, 0

        if self.today:
            if today is None:
                today = datetime.now().date()
            # Check if its a new day, then refresh idx
            if today != self.today:
                self.today = today
                self._next_id = 1

        # Run matching cascade.
        matches, unmatched_tracks, unmatched_detections = self._match(detections)

        '''
        # visualize features of the detections and the tracks
        from sklearn.manifold import TSNE
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation

        feats_all, ids, cnt = [], [], 0
        for i, track in enumerate(self.tracks):
            if track.track_id in self.metric.samples:
                feats = self.metric.samples[track.track_id]
                if len(feats) > 0:
                    feats = np.concatenate([np.expand_dims(f, axis=0) for f in feats], axis=0)
                    print(feats.shape)
                    if len(feats) > 5:
                        feats = feats[-5: ]
                    feats_all.append(feats)
                    ids += [int(track.track_id)] * len(feats)
                    cnt = max(cnt, int(track.track_id))
                else:
                    print('len = 0, but exists')
            else:
                print('len = 0, does not exist')

        if len(feats_all) > 0:
            feats = np.concatenate(feats_all, axis=0)

            N, M = len(feats), len(detections)

            for det in detections:
                feats = np.concatenate([feats, np.expand_dims(det.feature, axis=0)], axis=0)
            print(N, M)

            print(ids)
            
            if len(feats) > 3:
                feats_embed = TSNE(n_components=2, init='random', perplexity=3).fit_transform(feats)
                # scale feate_embed to [0, 10]
                feats_embed[:, 0] = 10 * (feats_embed[:, 0] - feats_embed[:, 0].min()) / (feats_embed[:, 0].max() - feats_embed[:, 0].min() + 1e-6)
                feats_embed[:, 1] = 10 * (feats_embed[:, 1] - feats_embed[:, 1].min()) / (feats_embed[:, 1].max() - feats_embed[:, 1].min() + 1e-6)

                plt.cla()
                plt.scatter(feats_embed[:N, 0], feats_embed[:N, 1], c=np.array([[1, 1 / cnt * (i - 1), 1] for i in ids]))
                plt.scatter(feats_embed[N:, 0], feats_embed[N:, 1], c=np.array([[1 / M * i, 1, 0] for i in range(M)]))
                for i in range(N):
                    plt.annotate(str(ids[i]), (feats_embed[i][0], feats_embed[i][1]))
                for i in range(M):
                    plt.annotate('d' + str(i), (feats_embed[N + i][0], feats_embed[N + i][1]))
                
                plt.draw()
                plt.pause(0.001)
                # for i in range(N):
                #     plt.scatter(feats_embed[i][0], feats_embed[i][1], color=(255, 255, 255 // len(self.tracks) * colors[i]))

                # for i in range(N, N + M):
                #     plt.scatter(feats_embed[i][0], feats_embed[i][1], color=(255, 0, 255 // M * (i - N)))
        '''
        
        # Update track set.
        for track_idx, detection_idx in matches:
            self.tracks[track_idx].update(self.kf, detections[detection_idx])
        for track_idx in unmatched_tracks:
            self.tracks[track_idx].mark_missed()
        for detection_idx in unmatched_detections:
            if register_person and detections[detection_idx].register:
                self._initiate_track(detections[detection_idx])
                ret = 1, int(self.tracks[-1].track_id)

                print('initialized track')
                import cv2
                det = detections[detection_idx]
                x, y, w, h = det.ltwh
                x, y, w, h = int(x), int(y), int(w), int(h)
                # print(x, y, w, h, frame.shape)
                cv2.imshow('new track', frame[y: y + h, x: x + w])
                cv2.waitKey(1000)
            
        new_tracks = []
        self.del_tracks_ids = []
        for t in self.tracks:
            if not t.is_deleted():
                new_tracks.append(t)
            else:
                self.del_tracks_ids.append(t.track_id)
        self.tracks = new_tracks
        # self.tracks = [t for t in self.tracks if not t.is_deleted()]

        # Update distance metric.
        active_targets = [t.track_id for t in self.tracks if t.is_confirmed()]
        features, targets = [], []
        for track in self.tracks:
            if not track.is_confirmed():
                continue
            features += track.features
            targets += [track.track_id for _ in track.features]
            track.features = []
        self.metric.partial_fit(
            np.asarray(features), np.asarray(targets), active_targets
        )

        return matches, ret

    def _match(self, detections):
        def gated_metric(tracks, dets, track_indices, detection_indices):
            features = np.array([dets[i].feature for i in detection_indices])
            targets = np.array([tracks[i].track_id for i in track_indices])
            cost_matrix = self.metric.distance(features, targets)
            # gate the cost matrix with distance
            # cost_matrix[cost_matrix > ]

            print(f'Cost matrix: (thres = {self.metric.matching_threshold:.4f})')
            print(cost_matrix)
            cost_matrix = linear_assignment.gate_cost_matrix(
                self.kf, cost_matrix, tracks, dets, track_indices, detection_indices, only_position=self.gating_only_position
            )

            return cost_matrix

        # Split track set into confirmed and unconfirmed tracks.
        confirmed_tracks = [i for i, t in enumerate(self.tracks) if t.is_confirmed()]
        unconfirmed_tracks = [
            i for i, t in enumerate(self.tracks) if not t.is_confirmed()
        ]

        # Associate confirmed tracks using appearance features.
        (
            matches_a,
            unmatched_tracks_a,
            unmatched_detections,
        ) = linear_assignment.matching_cascade(
            gated_metric,
            self.metric.matching_threshold,
            self.max_age,
            self.tracks,
            detections,
            confirmed_tracks,
        )

        # Associate remaining tracks together with unconfirmed tracks using IOU.
        iou_track_candidates = unconfirmed_tracks + [
            k for k in unmatched_tracks_a if self.tracks[k].time_since_update == 1
        ]
        unmatched_tracks_a = [
            k for k in unmatched_tracks_a if self.tracks[k].time_since_update != 1
        ]
        (
            matches_b,
            unmatched_tracks_b,
            unmatched_detections,
        ) = linear_assignment.min_cost_matching(
            iou_matching.iou_cost,
            self.max_iou_distance,
            self.tracks,
            detections,
            iou_track_candidates,
            unmatched_detections,
        )

        matches = matches_a + matches_b
        unmatched_tracks = list(set(unmatched_tracks_a + unmatched_tracks_b))
        return matches, unmatched_tracks, unmatched_detections

    def _initiate_track(self, detection):
        mean, covariance = self.kf.initiate(detection.to_xyah())

        if self.today:
            track_id = "{}_{}".format(self.today, self._next_id)
        else:
            track_id = "{}".format(self._next_id)
        self.tracks.append(
            self.track_class(
                mean,
                covariance,
                track_id,
                self.n_init,
                self.max_age,
                # mean, covariance, self._next_id, self.n_init, self.max_age,
                feature=detection.feature,
                original_ltwh=detection.get_ltwh(),
                det_class=detection.class_name,
                det_conf=detection.confidence,
                instance_mask=detection.instance_mask,
                others=detection.others,
            )
        )
        self._next_id += 1

    def delete_all_tracks(self):
        self.tracks = []
        self._next_id = 1
