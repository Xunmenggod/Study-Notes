import pinocchio as pin
import numpy as np

if __name__ == "__main__":
    model = pin.buildModelFromMJCF("kuka_iiwa_14/iiwa14.xml",
                                root_joint=pin.JointModelFreeFlyer())
    fixed_model = pin.buildModelFromMJCF("kuka_iiwa_14/iiwa14.xml")
    fixed_data = fixed_model.createData()
    data = model.createData()
    print(f'nq: {model.nq}, nv: {model.nv}')
    for i in range(model.nframes):
        print(f'{i}th frame name: {model.frames[i].name}')
    ee_name = "link7"
    ee_id = model.getFrameId(ee_name)
    print(f'eeid: {ee_id}')
    # for i in range(model.njoints):
    #     print(f'{i}th joint: {model.joints[i]}')
        
    q = pin.randomConfiguration(model)
    fixed_q = q[7:]
    q[:7] = [0 ,0, 0, 1, 0, 0, 0]
    print(f'q: {q}')
    print(f'h: {data.nle}')
    print(f'ee pose: {data.oMf[ee_id]}')
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pin.computeAllTerms(model, data, q, np.zeros(model.nv))
    print(f'ee pose: {data.oMf[ee_id]}')
    print(f'h: {data.nle}, h shape: {data.nle.shape}')
    # LOCAL, WORLD
    J = pin.computeFrameJacobian(model, data, q, frame_id=ee_id,
                             reference_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    print(f'J: {J}, J dimension: {J.shape}')
    fixed_J = pin.computeFrameJacobian(fixed_model, fixed_data, fixed_q, ee_id, 
                                 pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    print(f'J fixed model: {fixed_J}, J dimension: {fixed_J.shape}')
    residue = fixed_J - J[:, 6:]
    residue_val = np.sum(residue, axis=1)
    print(f'residue shape: {residue_val.shape}')
    residue_val = np.linalg.norm(residue_val)
    print(f'residue: {residue}, value: {residue_val}')
    tau = pin.rnea(model, data, q, np.zeros(model.nv), np.zeros(model.nv))
    print(f'taue: {tau}')
    J_dot = 