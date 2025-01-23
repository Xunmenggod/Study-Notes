import mujoco
import mujoco.viewer
import numpy as np
import time
import osqp
from scipy import sparse

# Cartesian impedance control gains.
impedance_pos = np.asarray([100.0, 100.0, 100.0])  # [N/m]
impedance_ori = np.asarray([50.0, 50.0, 50.0])  # [Nm/rad]

# Joint impedance control gains.
Kp_null = np.asarray([75.0, 75.0, 50.0, 50.0, 40.0, 25.0, 25.0])

# Damping ratio for both Cartesian and joint impedance control.
damping_ratio = 1

# Gains for the twist computation. These should be between 0 and 1. 0 means no
# movement, 1 means move the end-effector to the target in one integration step.
Kpos: float = 0.95

# Gain for the orientation component of the twist computation. This should be
# between 0 and 1. 0 means no movement, 1 means move the end-effector to the target
# orientation in one integration step.
Kori: float = 0.95

# Integration timestep in seconds.
integration_dt: float = 1.0

# Whether to enable gravity compensation.
gravity_compensation: bool = True

# Simulation timestep in seconds.
dt: float = 0.002


def main() -> None:
    assert mujoco.__version__ >= "3.1.0", "Please upgrade to mujoco 3.1.0 or later."

    # Load the model and data.
    model = mujoco.MjModel.from_xml_path("kuka_iiwa_14/scene.xml")
    data = mujoco.MjData(model)

    model.opt.timestep = dt

    # Compute damping and stiffness matrices.
    damping_pos = damping_ratio * 2 * np.sqrt(impedance_pos)
    damping_ori = damping_ratio * 2 * np.sqrt(impedance_ori)
    Kp = np.concatenate([impedance_pos, impedance_ori], axis=0)
    Kd = np.concatenate([damping_pos, damping_ori], axis=0)
    Kd_null = damping_ratio * 2 * np.sqrt(Kp_null)

    # End-effector site we wish to control.
    site_name = "attachment_site"
    site_id = model.site(site_name).id

    # Get the dof and actuator ids for the joints we wish to control. These are copied
    # from the XML file. Feel free to comment out some joints to see the effect on
    # the controller.
    joint_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
    ]
    dof_ids = np.array([model.joint(name).id for name in joint_names])
    actuator_ids = np.array([model.actuator(name).id for name in joint_names])

    # Initial joint configuration saved as a keyframe in the XML file.
    key_name = "home"
    key_id = model.key(key_name).id
    q0 = model.key(key_name).qpos

    # Mocap body we will control with our mouse.
    mocap_name = "target"
    mocap_id = model.body(mocap_name).mocapid[0]

    # Pre-allocate numpy arrays.
    jac = np.zeros((6, model.nv))
    twist = np.zeros(6)
    site_quat = np.zeros(4)
    site_quat_conj = np.zeros(4)
    error_quat = np.zeros(4)
    M_inv = np.zeros((model.nv, model.nv))
    Mx = np.zeros((6, 6))

    print(f'model nq nv nu {model.nq, model.nv, model.nu}')
    
    solver = osqp.OSQP()
    is_first = True
    selection_mat = [1,0,0,0,0,0,0]

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=True,
        show_right_ui=False,
    ) as viewer:
        # Reset the simulation.
        mujoco.mj_resetDataKeyframe(model, data, key_id)

        # Reset the free camera.
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Enable site frame visualization.
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE
        while viewer.is_running():
            step_start = time.time()

            # Spatial velocity (aka twist).
            dx = data.mocap_pos[mocap_id] - data.site(site_id).xpos
            twist[:3] = Kpos * dx / integration_dt
            mujoco.mju_mat2Quat(site_quat, data.site(site_id).xmat)
            mujoco.mju_negQuat(site_quat_conj, site_quat)
            mujoco.mju_mulQuat(error_quat, data.mocap_quat[mocap_id], site_quat_conj)
            mujoco.mju_quat2Vel(twist[3:], error_quat, 1.0)
            ori_err = twist[3:]
            twist[3:] *= Kori / integration_dt
            pose_err = np.ones(6)
            pose_err[:3] = dx
            pose_err[3:] = ori_err
            twistErr = np.ones((6,1))
            # @TODO: sit type int
            mujoco.mj_objectVelocity(model, data,
                       7, site_id, twistErr, 1)
            stiffness_gain = np.eye(6)
            damping_gain = np.eye(6)
            gains = [300,300,300,20,20,20]
            for i in range(6):
                stiffness_gain[i,i] = gains[i]
            
            # Jacobian.
            mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)
            jac_dot = np.zeros((6,7))
            mujoco.mj_jacDot(model, data, jac_dot[:3], jac_dot[3:],
               np.array([0,0,0]), mujoco.mj_name2id(model, data, mujoco.mjtObj.mjOBJ_BODY, "attachment"))

            # Compute the task-space inertia matrix.
            mujoco.mj_solveM(model, data, M_inv, np.eye(model.nv))
            print(f"M inv: {M_inv.shape}\n{M_inv}")
            Mx_inv = jac @ M_inv @ jac.T
            if abs(np.linalg.det(Mx_inv)) >= 1e-2:
                Mx = np.linalg.inv(Mx_inv)
            else:
                Mx = np.linalg.pinv(Mx_inv, rcond=1e-2)
            
            # np.linalg.s
            Mx_sqrt = np.linalg.svd(Mx, compute_uv=0)
            print(f'svd Mx: \n{Mx_sqrt}')
            Mx_sqrt = np.sqrt(Mx_sqrt)
            close_loop_gain = Mx_sqrt @ np.sqrt(stiffness_gain) +  np.sqrt(stiffness_gain) @ Mx_sqrt
            print(f"mx: \n {Mx}cl damping:\n {close_loop_gain}")
            for i in range(6):
                damping_gain[i,i] = close_loop_gain[i]
            cur_q  = data.qpos[dof_ids]
            cur_dq = data.qvel[dof_ids]
            hessian = jac.transpose()@jac
            hessian = sparse.csc_matrix(hessian)
            jdot_qdot = 1
            gradient = jac.transpose() @ (jdot_qdot - Mx_inv@(stiffness_gain @ pose_err + damping_gain @ twistErr))
            
            acc = stiffness_gain @ pose_err + damping_gain @ twistErr
            des_ddq = np.linalg.pinv(jac) * (acc - jac_dot @ cur_dq)
            
            
            dof = 7
            qMin = -3.14*np.ones(7)
            qMax = 3.14*np.ones(7)
            dqMin = -9 * np.ones(7)
            dqMax = 9 * np.ones(7)
            tauMin = -20 * np.ones(7)
            tauMax = 20 * np.ones(7)
            Idof = np.eye(dof)
            accMat = (1 / 2) * dt * dt * Idof
            velMat = dt * Idof
            Ac = np.zeros((3*dof, dof))
            Ac[:dof] = accMat
            Ac[dof:2*dof] = velMat
            Ac[2*dof:] = np.linalg.pinv(M_inv)
            Ac = sparse.csc_matrix(Ac)
            # // update lb & ub
            lb = np.ones(3*7)
            lb[0:dof] = qMin - dt * cur_dq - cur_q
            lb[dof:dof+dof] = dqMin - cur_dq
            lb[2 * dof: 2 * dof + dof] = tauMin - data.qfrc_bias[dof_ids]
            ub = np.ones(3*7)
            ub[0:dof] = qMax - dt * cur_dq - cur_q
            ub[dof:dof+dof] = dqMax - cur_dq
            ub[2 * dof: 2 * dof + dof] = tauMax - data.qfrc_bias[dof_ids]
            if is_first:
                solver.setup(hessian, gradient, Ac, lb, ub)
                is_first = False
            else:
                solver.update(gradient,lb, ub, Px=hessian, Ax=Ac)
                
            solution = solver.solve()
            solution = solution.x
            print(f'soplution: {solution}')
            # tau = np.linalg.pinv(M_inv) @ solution
            tau = np.linalg.pinv(M_inv) @ des_ddq

            # Compute generalized forces.
            # tau = jac.T @ Mx @ (Kp * twist - Kd * (jac @ data.qvel[dof_ids]))

            # Add joint task in nullspace, keep the key joint position
            # Jbar = M_inv @ jac.T @ Mx
            # ddq = Kp_null * (q0 - data.qpos[dof_ids]) - Kd_null * data.qvel[dof_ids]
            # tau += (np.eye(model.nv) - jac.T @ Jbar.T) @ ddq

            # Add gravity compensation.
            if gravity_compensation:
                tau += data.qfrc_bias[dof_ids]
                print(f'tau: {tau[0]}')

            # Set the control signal and step the simulation.
            np.clip(tau, *model.actuator_ctrlrange.T, out=tau)
            data.ctrl[actuator_ids] = tau[actuator_ids]
            mujoco.mj_step(model, data)

            viewer.sync()
            time_until_next_step = dt - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()