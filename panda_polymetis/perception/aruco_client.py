""" Run a camera in a process/thread so we can try to have the best estimate possible for aruco tag poses. """

import multiprocessing as mp
from multiprocessing import shared_memory
import threading
import queue
import time
import copy
import atexit
import signal

import numpy as np
import cv2
import cv2.aruco as aruco

from panda_polymetis.perception.realsense_client import RealsenseAPI
from transform_utils.pose_transforms import PoseTransformer


class ArucoClient:
    def __init__(
        self,
        height,
        width,
        valid_marker_ids=(0,),
        marker_width=.025,  # metres
        dictionary='DICT_4X4_50',
        max_marker_stale_time=.5,  # seconds
        base_to_cam_tf=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # tvec + euler sxyz
        marker_to_obj_tf=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ):

        # use this if we need to expose intrinsics
        # temp_cam = RealsenseAPI(height=height, width=width, fps=30, warm_start=30)
        # intrinsics = temp_cam.get_intrinsics()
        # temp_cam.close()

        self._height = height
        self._width = width
        self.max_marker_stale_time = max_marker_stale_time
        if base_to_cam_tf != [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]:
            self.base_to_cam_tf_mat = PoseTransformer(
                pose=base_to_cam_tf, rotation_representation='euler', axes='sxyz').get_matrix()
        if marker_to_obj_tf != [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]:
            self.marker_to_obj_tf_mat = PoseTransformer(
                pose=marker_to_obj_tf, rotation_representation='euler', axes='sxyz').get_matrix()

        # aruco info to keep track of
        self._dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
        self.valid_marker_ids = list(valid_marker_ids)
        assert len(valid_marker_ids) > 0
        self.marker_width = marker_width
        dummy_poses = np.zeros([len(valid_marker_ids), 6], dtype=np.float32)
        self._poses_shm = shared_memory.SharedMemory(create=True, size=dummy_poses.nbytes)
        self._marker_poses = np.ndarray([len(valid_marker_ids), 6], dtype=np.float32, buffer=self._poses_shm.buf)
        # self._marker_poses = np.ndarray([len(valid_marker_ids), 6], dtype=np.float32)
        self._ret_marker_poses = np.ndarray([len(valid_marker_ids), 6])
        self._noshm_poses = dict()
        self._pose_lock = mp.Lock()

        # set up multiprocessing and shared memory
        dummy = np.zeros([height, width, 3], dtype=np.uint8)
        self._img_shm = shared_memory.SharedMemory(create=True, size=dummy.nbytes)
        self._latest_img = np.ndarray([height, width, 3], dtype=np.uint8, buffer=self._img_shm.buf)
        self._ret_latest_img = np.ndarray([height, width, 3], dtype=np.uint8)
        self._img_lock = mp.Lock()
        self._comm_q_in = mp.Queue()
        self._comm_q_out = mp.Queue()

        self.cam_p = mp.Process(target=self._reader, args=(
            self._img_shm.name, self._img_lock, self._poses_shm.name, self._pose_lock,
            self._comm_q_in, self._comm_q_out))
        self.cam_p.daemon = True
        self.cam_p.start()

        # can't get these to work, so leaving out...results in a shared memory leak warning
        # atexit.register(self.exit_handler)
        # signal.signal(signal.SIGINT, self.handle_signal)
        # signal.signal(signal.SIGTERM, self.handle_signal)

        # self.cam_q = queue.Queue()
        # self.cam_p = threading.Thread(target=self._reader, args=(self.cam_q,))
        # self.cam_p.daemon = True
        # self.cam_p.start()

        # while self._latest_img is None:
        #     time.sleep(.001)
        self._comm_q_out.get(block=True)

    def _reader(self, img_shm_name: str, img_lock: mp.Lock, pose_shm_name: str, pose_lock: mp.Lock,
                q_in: mp.Queue, q_out: mp.Queue):
        cam = RealsenseAPI(height=self._height, width=self._width, fps=30, warm_start=60)

        # shared memory image with self._latest_img
        # you actually, technically, don't need these (it works without them), but it's much more unintuitive
        # without them.
        img_shm = shared_memory.SharedMemory(name=img_shm_name)
        img = np.ndarray([self._height, self._width, 3], dtype=np.uint8, buffer=img_shm.buf)
        pose_shm = shared_memory.SharedMemory(name=pose_shm_name)
        poses = np.ndarray([len(self.valid_marker_ids), 6], dtype=np.float32, buffer=pose_shm.buf)

        # get camera intrinsics
        intrinsics = cam.get_intrinsics()
        cam_mat = np.eye(3)
        cam_mat[0, 0] = intrinsics[0].fx
        cam_mat[1, 1] = intrinsics[0].fy
        cam_mat[0, 2] = intrinsics[0].ppx
        cam_mat[1, 2] = intrinsics[0].ppy
        distort = np.zeros([1, 5])
        distort[:] = intrinsics[0].coeffs

        # aruco detector
        # ar_params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(dictionary=self._dictionary)

        # marker stale detection
        last_marker_time = dict()
        for id in self.valid_marker_ids:
            last_marker_time[id] = time.time()

        last_time = time.time()

        q_out.put('init')

        while True:
            cam_img = cam.get_rgb(reverse_channels=True)
            img_lock.acquire()
            img[:] = cam_img[:]
            # img_lock.release()

            # print(f"Image acquire time: {time.time() - last_time}")
            last_time = time.time()

            # compute aruco marker locations
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected_img_points = detector.detectMarkers(gray)
            marker_read_time = time.time()

            if len(corners) > 0:
                # img_lock.acquire()
                aruco.drawDetectedMarkers(img, corners, ids)
            img_lock.release()

            for id in last_marker_time.keys():
                if marker_read_time > last_marker_time[id] + self.max_marker_stale_time:
                    print(f"WARNING: marker {id} is stale for {marker_read_time - last_marker_time[id]:.3f}s")

            if len(corners) > 0:
                pose_lock.acquire()
                for id, corner in zip(ids, corners):
                    iid = int(id)
                    if iid in self.valid_marker_ids:
                        last_marker_time[iid] = time.time()
                        valid_i = self.valid_marker_ids.index(iid)
                        rvec, tvec, marker_points = aruco.estimatePoseSingleMarkers(
                            corner, self.marker_width, cam_mat, distort)
                        # self._marker_poses[valid_i] = np.concatenate([tvec.flatten(), rvec.flatten()])

                        if hasattr(self, 'base_to_cam_tf_mat') or hasattr(self, 'marker_to_obj_tf_mat'):
                            # start_trans = time.time()
                            marker_tf_mat = PoseTransformer(
                                pose=np.concatenate([tvec.flatten(), rvec.flatten()]),
                                rotation_representation='rvec').get_matrix()
                            if hasattr(self, 'base_to_cam_tf_mat'):
                                marker_tf_mat = self.base_to_cam_tf_mat.dot(marker_tf_mat)
                            if hasattr(self, 'marker_to_obj_tf_mat'):
                                marker_tf_mat = marker_tf_mat.dot(self.marker_to_obj_tf_mat)
                            pose_tf = PoseTransformer(pose=marker_tf_mat, rotation_representation='mat')
                            marker_tvec_rvec = pose_tf.get_array_rvec()
                            tvec = marker_tvec_rvec[:3]
                            rvec = marker_tvec_rvec[3:]

                            # if valid_i == 0:
                            #     print(f"trans time: {time.time() - start_trans:.4f}")

                        poses[valid_i] = np.concatenate([tvec.flatten(), rvec.flatten()])
                pose_lock.release()

            if not q_in.empty():
                comm = q_in.get(block=True)
                if comm == 'close':
                    break

        img_shm.close()
        pose_shm.close()
        print("Closed shared memory within process.")
        q_out.put('closed')

    def get_latest_image(self):
        self._img_lock.acquire()
        # acq_start = time.time()
        self._ret_latest_img[:] = self._latest_img[:]
        # print(f"img acq time: {time.time() - acq_start}")
        self._img_lock.release()
        return self._ret_latest_img

    def get_latest_poses(self):
        self._pose_lock.acquire()
        self._ret_marker_poses[:] = self._marker_poses[:]
        self._pose_lock.release()
        return self._ret_marker_poses

    def close_shm(self):
        self._comm_q_in.put('close')
        self._comm_q_out.get(block=True)
        self._img_shm.close()
        self._img_shm.unlink()
        self._poses_shm.close()
        self._poses_shm.unlink()
        print("Unlinked shared memory.")

    def get_rigid_body_rel_poses(self):
        """ Interact with user to get relative poses between fixed markers """
        raise NotImplementedError()

    def exit_handler(self):
        print("EXIT HANDLER")
        self.close_shm()

    # def handle_signal(self, signum, frame):
    #     print("SIGNAL")
    #     self.exit_handler()
    #     exit(0)

    # def _reader(self, q: queue.Queue):
    #     self._cam = RealsenseAPI(height=480, width=640, fps=30, warm_start=30)
    #     while True:
    #         img = self._cam.get_rgb(reverse_channels=True)
    #         # self._img_lock.acquire()
    #         self._latest_img = img
    #         # self._img_lock.release()

    #         while not q.empty():  # clear so get_image only gets latest
    #             q.get(block=False)
    #         q.put(img)

    # def get_image(self, block_until_latest=True):
    #     if block_until_latest:
    #         img = self.cam_q.get()
    #         return img
    #     else:
    #         # self._img_lock.acquire()
    #         img = self._latest_img
    #         # self._img_lock.release()
    #         return img



if __name__ == '__main__':
    ac = ArucoClient(480, 640, marker_width=.052, valid_marker_ids=(0, 1),
        base_to_cam_tf=[.2, .1, .1, 0, 0, 0], marker_to_obj_tf=[.3, 0., 0., 0., 0., 0.])

    try:
        for _ in range(50):
            # img = ac.get_image(block_until_latest=False)
            img = ac.get_latest_image()
            cv2.imshow('Estimated Pose', img)

            poses = ac.get_latest_poses()
            print(poses)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            time.sleep(.2)

    finally:
        ac.close_shm()