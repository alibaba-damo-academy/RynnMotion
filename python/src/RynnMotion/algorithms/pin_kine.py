import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R


class PinKine:
    """
    Pinocchio-based kinematics solver with multi-site support.

    Supports both kinematic sites (on robot joints) and world-fixed sites (cameras, sensors).

    World-Fixed Site Convention:
        For sites attached to world (no joint ancestors), use matching body/site names:
        <body name="camera_front" pos="x y z" euler="rx ry rz">
          <site name="camera_front" />  <!-- No offset, inherits body transform -->
        </body>

        Pinocchio will use the BODY frame which correctly captures the fixed transform.

    Args:
        MJCF_path (str): Path to MJCF model file
        site_names (list[str] | None): List of site names to track
            - None: No-site mode (kinematics only, no sites)
            - List: Multi-site mode (e.g., ["shoulderSite", "EE", "camera_front"])
    """

    def __init__(self, MJCF_path, site_names=None):
        # Load Pinocchio model from MJCF
        self.pin_model, self.collision_model, self.visual_model = (
            pin.buildModelsFromMJCF(MJCF_path)
        )
        self.pin_data = self.pin_model.createData()
        self.q = None

        self.q_ref = None
        self.q_min = self.pin_model.lowerPositionLimit.copy()
        self.q_max = self.pin_model.upperPositionLimit.copy()
        self.null_kp = 0.1

        # Parse site names
        if site_names is None or (
            isinstance(site_names, list) and len(site_names) == 0
        ):
            # No-site mode
            self._site_names = []
        elif isinstance(site_names, list):
            # Multi-site mode
            self._site_names = site_names
        else:
            raise TypeError(
                f"site_names must be None or list[str], got {type(site_names)}"
            )

        # Initialize site data structures
        self._init_sites()

        self.get_endeffector_active_joints()

        # Identify end-effector sites (names starting with 'EE' or containing '_EE')
        self._identify_end_effector_sites()

        if len(self._ee_indices) == 0 and len(self._site_names) > 0:
            print(
                "Warning: No end-effector sites detected. "
                "Site names should start with 'EE'/'ee' or contain '_EE'/'_ee'."
            )

    def _init_sites(self):
        """
        Initialize site frame IDs and data storage.

        Frame Selection Logic:
        - Prefers BODY frames over OP_FRAME (works for world-fixed sites with matching names)
        - Falls back to OP_FRAME for kinematic sites
        """
        if len(self._site_names) == 0:
            # No-site mode
            self._site_frame_ids = []
            self._sitePos = []
            self._siteQuat = []
            self._siteJaco = []
            # self._site_ee_ids = []
            return

        # Find frame ID for each site
        self._site_frame_ids = []
        # self._site_ee_ids = []
        for site_name in self._site_names:
            try:
                # Search for matching frames by name
                body_frame_id = None
                op_frame_id = None

                for fid in range(self.pin_model.nframes):
                    frame = self.pin_model.frames[fid]
                    if frame.name == site_name:
                        if frame.type == pin.FrameType.BODY:
                            body_frame_id = fid
                        elif frame.type == pin.FrameType.OP_FRAME:
                            op_frame_id = fid

                # Prefer BODY frame (handles world-fixed sites with body name = site name)
                # Fall back to OP_FRAME (typical for kinematic sites)
                if body_frame_id is not None:
                    frame_id = body_frame_id
                elif op_frame_id is not None:
                    frame_id = op_frame_id
                else:
                    raise ValueError(
                        f"No BODY or OP_FRAME found for site '{site_name}'"
                    )

                self._site_frame_ids.append(frame_id)

            except Exception as e:
                raise ValueError(f"Site '{site_name}' not found in model: {e}")

        # Allocate storage for site data
        num_sites = len(self._site_names)
        self._sitePos = [None] * num_sites
        self._siteQuat = [None] * num_sites
        self._siteJaco = [None] * num_sites

    def update(self, q):
        """
        Update forward kinematics with joint configuration.

        Args:
            q (np.ndarray): Joint position vector
        """
        self.q = q
        pin.forwardKinematics(self.pin_model, self.pin_data, self.q)
        pin.updateFramePlacements(self.pin_model, self.pin_data)

        if len(self._site_names) == 0:
            return  # No sites to update

        # Update all sites
        for i in range(len(self._site_names)):
            site_frame_id = self._site_frame_ids[i]

            # Get site transform from Pinocchio
            site_transform = self.pin_data.oMf[site_frame_id]

            # Extract position
            self._sitePos[i] = site_transform.translation.copy()

            # Extract quaternion (x, y, z, w) order
            quat = pin.Quaternion(site_transform.rotation).coeffs()
            self._siteQuat[i] = quat.copy()

            # Compute Jacobian in world frame
            self._siteJaco[i] = pin.computeFrameJacobian(
                self.pin_model, self.pin_data, self.q, site_frame_id, pin.WORLD
            )

    # ========== Query Methods ==========

    def getNumSites(self):
        """
        Get the number of sites.

        Returns:
            int: Number of sites (0 for no-site mode, 1 for single-site, N for multi-site)
        """
        return len(self._site_names)

    def getSiteName(self, site_index):
        """
        Get the name of a site by index.

        Args:
            site_index (int): Index of the site

        Returns:
            str: Site name

        Raises:
            IndexError: If site_index is out of range
        """
        if site_index < 0 or site_index >= len(self._site_names):
            raise IndexError(
                f"Site index {site_index} out of range (have {len(self._site_names)} sites)"
            )
        return self._site_names[site_index]

    def getSiteIndex(self, site_name):
        """
        Get the index of a site by name.

        Args:
            site_name (str): Name of the site

        Returns:
            int: Site index, or -1 if not found
        """
        try:
            return self._site_names.index(site_name)
        except ValueError:
            return -1

    # ========== Site Access by Index ==========

    def getSitePos(self, site_index):
        """
        Get the position of a specific site by index.

        Args:
            site_index (int): Index of the site

        Returns:
            np.ndarray: Site position vector (3,)

        Raises:
            IndexError: If site_index is out of range
            RuntimeError: If update() has not been called
        """
        if site_index < 0 or site_index >= len(self._site_names):
            raise IndexError(f"Site index {site_index} out of range")
        if self._sitePos[site_index] is None:
            raise RuntimeError("Must call update() before accessing site data")
        return self._sitePos[site_index]

    def getSiteQuat(self, site_index):
        """
        Get the orientation quaternion of a specific site by index.

        Args:
            site_index (int): Index of the site

        Returns:
            np.ndarray: Site orientation quaternion in (x, y, z, w) order (4,)

        Raises:
            IndexError: If site_index is out of range
            RuntimeError: If update() has not been called
        """
        if site_index < 0 or site_index >= len(self._site_names):
            raise IndexError(f"Site index {site_index} out of range")
        if self._siteQuat[site_index] is None:
            raise RuntimeError("Must call update() before accessing site data")
        return self._siteQuat[site_index]

    def getSiteJacobian(self, site_index):
        """
        Get the Jacobian of a specific site by index.

        Args:
            site_index (int): Index of the site

        Returns:
            np.ndarray: Site Jacobian matrix (6, nv)

        Raises:
            IndexError: If site_index is out of range
            RuntimeError: If update() has not been called
        """
        if site_index < 0 or site_index >= len(self._site_names):
            raise IndexError(f"Site index {site_index} out of range")
        if self._siteJaco[site_index] is None:
            raise RuntimeError("Must call update() before accessing site data")
        return self._siteJaco[site_index]

    # ========== Site Access by Name ==========

    def getSitePosByName(self, site_name):
        """
        Get the position of a specific site by name.

        Args:
            site_name (str): Name of the site

        Returns:
            np.ndarray: Site position vector (3,)

        Raises:
            ValueError: If site name not found
        """
        index = self.getSiteIndex(site_name)
        if index < 0:
            raise ValueError(f"Site '{site_name}' not found")
        return self.getSitePos(index)

    def getSiteQuatByName(self, site_name):
        """
        Get the orientation quaternion of a specific site by name.

        Args:
            site_name (str): Name of the site

        Returns:
            np.ndarray: Site orientation quaternion in (x, y, z, w) order (4,)

        Raises:
            ValueError: If site name not found
        """
        index = self.getSiteIndex(site_name)
        if index < 0:
            raise ValueError(f"Site '{site_name}' not found")
        return self.getSiteQuat(index)

    def getSiteSE3ByName(self, site_name):
        """
        Get the SE3 pose of a specific site by name.

        Args:
            site_name (str): Name of the site

        Returns:
            pin.SE3: Site pose as Pinocchio SE3 object

        Raises:
            ValueError: If site name not found
        """
        index = self.getSiteIndex(site_name)
        if index < 0:
            raise ValueError(f"Site '{site_name}' not found")

        pos = self.getSitePos(index)
        quat = self.getSiteQuat(index)

        rotation = pin.Quaternion(quat).toRotationMatrix()

        return pin.SE3(rotation, pos)

    def getSiteJacoByName(self, site_name):
        """
        Get the Jacobian of a specific site by name.

        Args:
            site_name (str): Name of the site

        Returns:
            np.ndarray: Site Jacobian matrix (6, nv)

        Raises:
            ValueError: If site name not found
        """
        index = self.getSiteIndex(site_name)
        if index < 0:
            raise ValueError(f"Site '{site_name}' not found")
        return self.getSiteJacobian(index)

    # ========== Legacy Single-Site Methods (Backward Compatibility) ==========

    def getEEPos(self, ee_index=0):
        """
        Get end-effector position by EE index.

        Args:
            ee_index (int): Index of the end-effector (default: 0 for first/only EE)

        Returns:
            np.ndarray: End-effector position vector (3,)

        Raises:
            RuntimeError: If no sites configured
            IndexError: If ee_index is out of range
        """
        if len(self._site_names) == 0:
            raise RuntimeError("Cannot get EE position in no-site mode")

        if ee_index < 0 or ee_index >= len(self._ee_indices):
            raise IndexError(
                f"EE index {ee_index} out of range (have {len(self._ee_indices)} end-effectors)"
            )

        site_idx = self._ee_indices[ee_index]
        return self._sitePos[site_idx]

    def getEEQuat(self, ee_index=0):
        """
        Get end-effector quaternion by EE index.

        Args:
            ee_index (int): Index of the end-effector (default: 0 for first/only EE)

        Returns:
            np.ndarray: End-effector quaternion in (x, y, z, w) order (4,)

        Raises:
            RuntimeError: If no sites configured
            IndexError: If ee_index is out of range
        """
        if len(self._site_names) == 0:
            raise RuntimeError("Cannot get EE quaternion in no-site mode")

        if ee_index < 0 or ee_index >= len(self._ee_indices):
            raise IndexError(f"EE index {ee_index} out of range")

        site_idx = self._ee_indices[ee_index]
        return self._siteQuat[site_idx]

    def getEEPose(self, ee_index=0):
        """
        Get end-effector pose by EE index.

        Args:
            ee_index (int): Index of the end-effector (default: 0 for first/only EE)

        Returns:
            tuple: (position, quaternion) where quaternion is in (x, y, z, w) order
        """
        return self.getEEPos(ee_index), self.getEEQuat(ee_index)

    def getEEJaco(self, ee_index=0):
        """
        Get end-effector Jacobian matrix by EE index.

        Args:
            ee_index (int): Index of the end-effector (default: 0 for first/only EE)

        Returns:
            np.ndarray: 6xN Jacobian matrix in WORLD frame

        Raises:
            RuntimeError: If no sites configured
            IndexError: If ee_index is out of range
        """
        if len(self._site_names) == 0:
            raise RuntimeError("Cannot get EE Jacobian in no-site mode")

        if ee_index < 0 or ee_index >= len(self._ee_indices):
            raise IndexError(f"EE index {ee_index} out of range")

        site_idx = self._ee_indices[ee_index]
        return self._siteJaco[site_idx]

    # ========== End-Effector Detection Helper Methods ==========

    def _is_end_effector_site(self, site_name):
        """
        Check if a site name is an end-effector.
        Matches names starting with 'ee' or containing '_ee' (case-insensitive).
        Examples: 'EE', 'EE_01', 'eeSite_01', 'left_EE'
        Avoids false positives like 'knee', 'steering'.
        """
        lower = site_name.lower()

        # Check if starts with "ee"
        if lower.startswith("ee"):
            return True

        # Check if contains "_ee"
        if "_ee" in lower:
            return True

        return False

    def _identify_end_effector_sites(self):
        """Identify which sites are end-effectors based on naming convention."""
        self._ee_indices = []

        # Auto-detect EE sites by name (starts with 'EE' or contains '_EE')
        for i, site_name in enumerate(self._site_names):
            if self._is_end_effector_site(site_name):
                self._ee_indices.append(i)

    # ========== End-Effector Query Methods ==========

    def getNumEndEffectors(self):
        """
        Get the number of end-effectors.

        Returns:
            int: Number of end-effector sites (auto-detected by name)
        """
        return len(self._ee_indices)

    def hasEndEffectors(self):
        """
        Check if this PinKine has any end-effectors.

        Returns:
            bool: True if at least one end-effector exists
        """
        return len(self._ee_indices) > 0

    def getEESiteName(self, ee_index=0):
        """
        Get the name of a specific end-effector.

        Args:
            ee_index (int): Index of the end-effector

        Returns:
            str: End-effector site name (e.g., "EE", "EE_left", "eeSite_01")

        Raises:
            IndexError: If ee_index is out of range
        """
        if ee_index < 0 or ee_index >= len(self._ee_indices):
            raise IndexError(f"EE index {ee_index} out of range")

        site_idx = self._ee_indices[ee_index]
        return self._site_names[site_idx]

    def getSiteIndexFromEEIndex(self, ee_index):
        """
        Convert EE index to site index.

        Args:
            ee_index (int): Index in the end-effector list

        Returns:
            int: Index in the full site list

        Raises:
            IndexError: If ee_index is out of range
        """
        if ee_index < 0 or ee_index >= len(self._ee_indices):
            raise IndexError(f"EE index {ee_index} out of range")
        return self._ee_indices[ee_index]

    def getEEIndexFromSiteIndex(self, site_index):
        """
        Convert site index to EE index.

        Args:
            site_index (int): Index in the full site list

        Returns:
            int: Index in the end-effector list, or -1 if not an EE
        """
        try:
            return self._ee_indices.index(site_index)
        except ValueError:
            return -1  # Not an end-effector

    def get_endeffector_active_joints(self) -> np.ndarray:
        """
        Automatically extract the joint DOF indices that affect a given end-effector frame.

        Returns:
            np.ndarray: Array of q-space indices (e.g., [0,1,2,4,...]) that are in the kinematic chain to EE
        """
        self._sites_active_joint_ids = []

        for frame_id in self._site_frame_ids:

            frame = self.pin_model.frames[frame_id]
            current_joint = (
                frame.parentJoint
            )  # frame.parent  # The joint this frame is attached to

            joint_chain = []
            while current_joint > 0:  # 0 is universe/root
                joint_chain.append(current_joint)

                current_joint = self.pin_model.parents[current_joint]
                if current_joint <= 0:
                    break

            # reverse joint chain to go from base to end-effector
            joint_chain = joint_chain[::-1]

            # active joints only (skip fixed joints)
            active_joint_ids = []
            for joint_id in joint_chain:
                if self.pin_model.joints[joint_id].nq > 0:
                    active_joint_ids.append(joint_id)

            active_q_indices = []
            for joint_id in active_joint_ids:
                q_start_idx = self.pin_model.joints[joint_id].idx_q
                nq_joint = self.pin_model.joints[joint_id].nq

                active_q_indices.extend(range(q_start_idx, q_start_idx + nq_joint))

            self._sites_active_joint_ids.append(np.array(active_q_indices))
            # print(self._sites_active_joint_ids)

        return self._sites_active_joint_ids

    def set_q_ref(self, q_ref, null_kp=0.1):
        """
        Set reference joint configuration for null-space IK.

        Args:
            q_ref (np.ndarray): Reference joint position vector
            null_kp (float): Gain for null-space projection
        """
        q_ref = np.asarray(q_ref).flatten()

        if len(q_ref) == 0:
            raise ValueError("q_ref must have at least one element.")

        nq = len(self.q_min)  # actual robot DOF

        if len(q_ref) < nq:
            # if q_ref is shorter than robot DOF, pad with zeros
            padded_q = np.zeros(nq)
            padded_q[: len(q_ref)] = q_ref
            q_ref = padded_q
        else:
            # if q_ref is longer than robot DOF, truncate it
            q_ref = q_ref[:nq]
            self.q_ref = self._joint_saturation(q_ref)
            self.null_kp = null_kp

    def _joint_saturation(self, q):
        """
        Apply joint limits to the given joint configuration.

        Args:
            q (np.ndarray): Joint position vector

        Returns:
            np.ndarray: Saturated joint position vector
        """
        return np.clip(q, self.q_min, self.q_max)

    def single_site_ik_compute(self, pos, quat, q_init, site_ids=0):
        """
        calculate the inverse kinematics.

        Args:
            pos (np.ndarray): Position vector
            quat (np.ndarray): Quaternion vector (x, y, z, w)
            q_init (np.ndarray): Joint position vector
        """

        if len(self._site_names) == 0:
            return  # No sites to update

        max_iter = 100
        tol = 1e-4
        alpha = 0.1

        target_se3 = pin.SE3(R.from_quat(quat).as_matrix(), pos)  # scipy.quat (x,y,z,w)
        q = q_init.copy()

        err = np.zeros(6)
        # Update all sites
        for i in range(max_iter):
            q = self._joint_saturation(q)
            pin.forwardKinematics(self.pin_model, self.pin_data, q)
            pin.updateFramePlacements(self.pin_model, self.pin_data)
            current_se3 = self.pin_data.oMf[site_ids]  # world_base_frame

            err[0:3] = target_se3.translation - current_se3.translation
            err[3:6] = pin.log(
                target_se3.rotation @ np.linalg.inv(current_se3.rotation)
            )

            if np.linalg.norm(err) < tol:
                # print("Converged in {} iterations".format(i))
                break

            # if i == max_iter - 1:
            #    print("Max iterations reached")

            jaco = pin.computeFrameJacobian(
                self.pin_model,
                self.pin_data,
                q,
                site_ids,
                pin.LOCAL_WORLD_ALIGNED,  # WORLD frame
            )

            # dls_inv_jaco = jaco.T @ np.linalg.inv(jaco @ jaco.T + 1e-4 * np.eye(6))
            # null_jaco = jaco.T @ np.linalg.inv(jaco @ jaco.T + 1e-8 * np.eye(6))
            # dq = dls_inv_jaco @ err
            # if self.q_ref is not None:
            #    null_space_proj = np.eye(len(q)) - null_jaco @ jaco
            #    null_dq = (
            #        null_space_proj @ (self.q_ref - q) * self.null_kp
            #    )  # null space term
            #    dq = dq + null_dq

            # use SVD + DLS
            U, S, Vt = np.linalg.svd(jaco, full_matrices=False)
            # lambda_ = 1e-2  # damping factor
            lambda_min = 1e-4
            lambda_ = lambda_min + 1e-2 * (1 / (1 + max(S)))
            S_damped = S / (S**2 + lambda_**2)
            J_inv_dls = (Vt.T * S_damped) @ U.T

            # inverse kinematics step
            dq = J_inv_dls @ err

            # null space projection
            if self.q_ref is not None:
                null_proj = np.eye(len(q)) - J_inv_dls @ jaco
                dq_null = self.null_kp * null_proj @ (self.q_ref - q)
                dq += dq_null

            q = pin.integrate(self.pin_model, q, alpha * dq)

        return q

    def site_ik_compute(self, pos, quat, q_init):
        q_inv_list = []
        for site_id in self._site_frame_ids:
            q_inv = self.single_site_ik_compute(pos, quat, q_init, site_id)
            q_inv_list.append(q_inv)
        return q_inv_list
        # return self.single_site_ik_compute(pos, quat, q_init, self._site_frame_ids[0])
