import time
import random
import pybullet as p
import pybullet_data

def simulate_paths(chosen_paths, grid_size, static_obstacles):

    width, height = grid_size

    # --- PyBullet Setup ---
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.8)

    # Camera
    p.resetDebugVisualizerCamera(
        cameraDistance=12,
        cameraYaw=0,
        cameraPitch=-89,
        cameraTargetPosition=[width/2, height/2, 0]
    )

    p.loadURDF("plane.urdf")

    # --- Draw grid ---
    for x in range(width+1):
        p.addUserDebugLine([x,0,0.01],[x,height,0.01],[0,0,0],1)
    for y in range(height+1):
        p.addUserDebugLine([0,y,0.01],[width,y,0.01],[0,0,0],1)

    # --- Obstacles ---
    obs_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.45,0.45,0.1])
    obs_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.45,0.45,0.1], rgbaColor=[0.6,0.6,0.6,1])
    for ox, oy in static_obstacles:
        p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=obs_col,
            baseVisualShapeIndex=obs_vis,
            basePosition=[ox+0.5, oy+0.5, 0.1]
        )

    # --- Robots spawn ---
    robot_ids = {}
    colors = {}
    for pid, path in chosen_paths.items():
        x, y, t = path[0]
        c = (random.random(), random.random(), random.random(), 1)
        colors[pid] = c

        vis = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.25,
            rgbaColor=c
        )
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.25)
        rid = p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[x + 0.5, y + 0.5, 0.25]
        )
        robot_ids[pid] = rid

    # --- Simulation timing ---
    SMOOTH_STEPS = 20  # higher = smoother
    max_time = max(path[-1][2] for path in chosen_paths.values())

    print("\n=== START CORRECT SIMULATION ===\n")

    # Each global_t corresponds EXACTLY to the path timestep
    for global_t in range(max_time + 1):

        for smooth in range(SMOOTH_STEPS):

            for pid, path in chosen_paths.items():

                # If finished
                if global_t >= len(path) - 1:
                    continue

                (x1, y1, t1) = path[global_t]
                (x2, y2, t2) = path[global_t + 1]

                # Must match discrete time EXACTLY
                if t2 != global_t + 1:
                    continue  # wait

                alpha = smooth / SMOOTH_STEPS
                xs = (1 - alpha) * x1 + alpha * x2
                ys = (1 - alpha) * y1 + alpha * y2

                p.resetBasePositionAndOrientation(
                    robot_ids[pid],
                    [xs + 0.5, ys + 0.5, 0.25],
                    [0, 0, 0, 1]
                )

            p.stepSimulation()
            time.sleep(1/240)

    print("\n=== SIMULATION FINISHED ===")
    input("Press ENTER to exit...")
    p.disconnect()
