"""
Drone Physics Builder - Physics setup for F-11 drones.

Single responsibility: Configure rigid bodies, mass properties, articulations, and joints.
"""

from typing import Tuple

from pxr import Gf, UsdGeom, Sdf, UsdPhysics, PhysxSchema
import omni.isaac.core.utils.prims as prim_utils


class DronePhysicsBuilder:
    """
    Configures physics for F-11 drones.

    Responsibilities:
    - Rigid body setup
    - Mass and inertia configuration
    - Articulation root setup
    - Revolute joint creation for rotors
    - Drive API configuration
    """

    def __init__(self, stage):
        """
        Initialize DronePhysicsBuilder.

        Args:
            stage: USD stage
        """
        self.stage = stage

    def setup_articulation_root(
        self,
        prim_path: str,
        mass: float,
        inertia: Tuple[float, float, float],
        center_of_gravity: Tuple[float, float, float],
    ) -> None:
        """
        Configure base_link as articulation root with mass properties.

        Args:
            prim_path: Path to base_link prim
            mass: Total mass in kg
            inertia: Diagonal inertia (Ixx, Iyy, Izz) in kg*m^2
            center_of_gravity: CoG offset in meters
        """
        prim = self.stage.GetPrimAtPath(prim_path)

        # Apply rigid body and mass APIs
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(mass)
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*center_of_gravity))

        # Apply articulation root
        UsdPhysics.ArticulationRootAPI.Apply(prim)
        physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        physx_articulation.CreateEnabledSelfCollisionsAttr(False)

    def setup_rotor_rigid_body(
        self,
        rotor_path: str,
        rotor_mass: float,
        rotor_inertia: Tuple[float, float, float],
        position: Tuple[float, float, float],
    ) -> None:
        """
        Configure a rotor as a rigid body with mass properties.

        Args:
            rotor_path: Path to rotor prim
            rotor_mass: Rotor mass in kg
            rotor_inertia: Diagonal inertia in kg*m^2
            position: Position relative to parent
        """
        rotor_prim = self.stage.GetPrimAtPath(rotor_path)

        # Apply rigid body and mass APIs
        UsdPhysics.RigidBodyAPI.Apply(rotor_prim)
        mass_api = UsdPhysics.MassAPI.Apply(rotor_prim)
        mass_api.CreateMassAttr(rotor_mass)
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*rotor_inertia))

        # Position rotor
        rotor_xform = UsdGeom.Xformable(rotor_prim)
        rotor_xform.ClearXformOpOrder()
        rotor_xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*position)
        )

    def create_rotor_joint(
        self,
        joint_path: str,
        parent_path: str,
        rotor_path: str,
        rotor_position: Tuple[float, float, float],
        rotor_direction: int,
        joint_damping: float,
    ) -> None:
        """
        Create revolute joint for rotor actuation.

        Args:
            joint_path: Path for the joint
            parent_path: Parent body path (base_link)
            rotor_path: Child body path (rotor)
            rotor_position: Position in parent frame
            rotor_direction: Spin direction (1=CW, -1=CCW)
            joint_damping: Joint damping coefficient
        """
        # Ensure Joints parent exists
        joints_parent = "/".join(joint_path.split("/")[:-1])
        if not self.stage.GetPrimAtPath(joints_parent):
            prim_utils.create_prim(joints_parent, "Xform")

        # Create revolute joint
        joint = UsdPhysics.RevoluteJoint.Define(self.stage, joint_path)
        joint.CreateAxisAttr("Z")
        joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateBody1Rel().SetTargets([rotor_path])

        # Set local transforms for joint
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*rotor_position))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

        # No limits for continuous rotation
        joint.CreateLowerLimitAttr(-1e10)
        joint.CreateUpperLimitAttr(1e10)

        # Add angular drive for velocity control
        drive_api = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive_api.CreateTypeAttr("force")
        drive_api.CreateDampingAttr(joint_damping)
        drive_api.CreateStiffnessAttr(0.0)  # Pure velocity control
        drive_api.CreateMaxForceAttr(1000.0)  # High max force

        # Store rotor direction as custom attribute
        joint.GetPrim().CreateAttribute(
            "flyby:rotorDirection",
            Sdf.ValueTypeNames.Int
        ).Set(rotor_direction)

    def create_gimbal_joint(
        self,
        joint_path: str,
        parent_path: str,
        child_path: str,
        axis: str,
        lower_limit: float,
        upper_limit: float,
    ) -> None:
        """
        Create revolute joint for gimbal axis.

        Args:
            joint_path: Path for the joint
            parent_path: Parent body path
            child_path: Child body path
            axis: Joint axis ("X", "Y", or "Z")
            lower_limit: Lower limit in degrees
            upper_limit: Upper limit in degrees
        """
        joint = UsdPhysics.RevoluteJoint.Define(self.stage, joint_path)
        joint.CreateAxisAttr(axis)
        joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateBody1Rel().SetTargets([child_path])
        joint.CreateLowerLimitAttr(lower_limit)
        joint.CreateUpperLimitAttr(upper_limit)
