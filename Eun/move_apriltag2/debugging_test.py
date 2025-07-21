# Pose → 위치/회전행렬
position = [msg.position.x, msg.position.y, msg.position.z]
# 쿼터니언 → 회전행렬
quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]  # transforms3d: w, x, y, z
rot = transforms3d.quaternions.quat2mat(quat)
# IKPy 역기구학
angles = self.chain.inverse_kinematics(
    position,
    target_orientation=rot, 
)