import pybullet as p
import pybullet_data
import numpy as np
import os
from multiprocessing import Pool


def make_arena(arena_size=10, wall_height=1):
    wall_thickness = 0.5
    floor_collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[arena_size / 2, arena_size / 2, wall_thickness],
    )
    floor_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[arena_size / 2, arena_size / 2, wall_thickness],
        rgbaColor=[1, 1, 0, 1],
    )
    floor_body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=floor_collision_shape,
        baseVisualShapeIndex=floor_visual_shape,
        basePosition=[0, 0, -wall_thickness],
    )

    wall_collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[arena_size / 2, wall_thickness / 2, wall_height / 2],
    )
    wall_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[arena_size / 2, wall_thickness / 2, wall_height / 2],
        rgbaColor=[0.7, 0.7, 0.7, 1],
    )  # Gray walls

    # Create four walls
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape,
        baseVisualShapeIndex=wall_visual_shape,
        basePosition=[0, arena_size / 2, wall_height / 2],
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape,
        baseVisualShapeIndex=wall_visual_shape,
        basePosition=[0, -arena_size / 2, wall_height / 2],
    )

    wall_collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[wall_thickness / 2, arena_size / 2, wall_height / 2],
    )
    wall_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[wall_thickness / 2, arena_size / 2, wall_height / 2],
        rgbaColor=[0.7, 0.7, 0.7, 1],
    )  # Gray walls

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape,
        baseVisualShapeIndex=wall_visual_shape,
        basePosition=[arena_size / 2, 0, wall_height / 2],
    )
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=wall_collision_shape,
        baseVisualShapeIndex=wall_visual_shape,
        basePosition=[-arena_size / 2, 0, wall_height / 2],
    )


class Simulation:
    def __init__(self, sim_id=0):
        self.physicsClientId = p.connect(p.DIRECT)
        self.sim_id = sim_id

    def capture_screenshot(self, filename, target_pos=(0, 0, 0)):
        """Capture a screenshot from the simulation."""
        pid = self.physicsClientId
        width, height = 640, 480

        # Camera positioned to view the creature and mountain
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=[8, 30, 6],
            cameraTargetPosition=target_pos,
            cameraUpVector=[0, 0, 1],
            physicsClientId=pid,
        )
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=width / height, nearVal=0.1, farVal=100, physicsClientId=pid
        )

        _, _, rgb, _, _ = p.getCameraImage(
            width,
            height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            physicsClientId=pid,
        )

        # Convert flat buffer to numpy array and reshape
        rgb = np.array(rgb, dtype=np.uint8).reshape(height, width, 4)

        # Save using PIL (avoids cv2 BGR conversion)
        from PIL import Image

        img = Image.fromarray(rgb[:, :, :3])
        img.save(filename)

    def run_creature(self, cr, iterations=9600, debug=False):
        pid = self.physicsClientId
        p.resetSimulation(physicsClientId=pid)
        p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=pid)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        arena_size = 40
        make_arena(arena_size=arena_size)

        p.setGravity(0, 0, -10, physicsClientId=pid)

        mountain_position = (0, 0, -1)  # Adjust as needed
        mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
        # p.setAdditionalSearchPath("./src/shapes/")
        # mountain = p.loadURDF("mountain.urdf", mountain_position, mountain_orientation, useFixedBase=1)
        # mountain = p.loadURDF("mountain_with_cubes.urdf", mountain_position, mountain_orientation, useFixedBase=1)

        mountain = p.loadURDF(
            "./src/shapes/gaussian_pyramid.urdf",
            mountain_position,
            mountain_orientation,
            useFixedBase=1,
        )

        xml_file = f"/dev/shm/creature_{self.sim_id}_{os.getpid()}.urdf"
        xml_str = cr.to_xml()
        with open(xml_file, "w") as f:
            f.write(xml_str)

        try:
            cid = p.loadURDF(xml_file, physicsClientId=pid)
        finally:
            os.remove(xml_file)

        p.resetBasePositionAndOrientation(
            cid, [5, 5, 10], [0, 0, 0, 1], physicsClientId=pid
        )

        peak_xy = np.asarray([0.0, 0.0])
        base_z = 0.0
        start_xy_dist = None
        best_z = -1e9
        height_sum = 0.0
        fell = False
        xy_dist = None
        steps_run = 0

        for step in range(iterations):
            p.stepSimulation(physicsClientId=pid)
            if step % 24 == 0:
                self.update_motors(cid=cid, cr=cr)

            pos, orn = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
            cr.update_position(pos)

            # Capture screenshot every 240 steps (~1 sec at 240fps)
            if debug and step % 240 == 0:
                self.capture_screenshot(
                    f"debug_{self.sim_id}_{step:04d}.png", target_pos=pos
                )
            # print(cr.get_distance_travelled())

            if start_xy_dist is None:
                start_xy_dist = np.linalg.norm(np.asarray(pos[:2]) - peak_xy)

            best_z = max(best_z, pos[2])
            height_sum += pos[2]
            xy_dist = np.linalg.norm(np.asarray(pos[:2]) - peak_xy)
            steps_run += 1

            if pos[2] < base_z - 0.5:
                fell = True
                break

        steps_run = max(1, steps_run)
        climb_score = max(0.0, best_z - base_z)
        approach_score = max(
            0.0, (start_xy_dist or 0.0) - (xy_dist if xy_dist is not None else 0.0)
        )
        height_time = height_sum / steps_run
        penalty = 1.0 if fell else 0.0
        cr.fitness_score = (
            climb_score + 0.5 * approach_score + 0.1 * height_time - penalty
        )

    def update_motors(self, cid, cr):
        """
        cid is the id in the physics engine
        cr is a creature object
        """
        for jid in range(p.getNumJoints(cid, physicsClientId=self.physicsClientId)):
            m = cr.get_motors()[jid]

            p.setJointMotorControl2(
                cid,
                jid,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=m.get_output(),
                force=5,
                physicsClientId=self.physicsClientId,
            )

    # You can add this to the Simulation class:
    def eval_population(self, pop, iterations):
        for cr in pop.creatures:
            self.run_creature(cr, 2400)


class ThreadedSim:
    def __init__(self, pool_size):
        self.sims = [Simulation(i) for i in range(pool_size)]
        self.pool_size = pool_size
        self.pool = Pool(pool_size)

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        sim.run_creature(cr, iterations)
        return cr

    def eval_population(self, pop, iterations):
        """
        pop is a Population object
        iterations is frames in pybullet to run for at 240fps
        """
        all_args = [
            (self.sims[i % len(self.sims)], cr, iterations)
            for i, cr in enumerate(pop.creatures)
        ]
        pop.creatures = self.pool.starmap(ThreadedSim.static_run_creature, all_args)

    def close(self):
        self.pool.close()
        self.pool.join()
