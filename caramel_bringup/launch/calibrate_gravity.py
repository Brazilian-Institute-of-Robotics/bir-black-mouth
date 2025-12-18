import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data # <--- IMPORTANTE
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import time

# CONFIGURAÇÃO
KP_ATUAL = 4.0  
SAMPLES = 500   

class GravityCalibrator(Node):
    def __init__(self):
        super().__init__('gravity_calibrator')
        
        self.legs = ['front_right', 'front_left', 'back_right', 'back_left']
        self.joint_suffixes = ['hip_roll_joint', 'hip_pitch_joint', 'elbow_joint']
        
        self.data = {leg: [[], [], []] for leg in self.legs}
        self.counts = {leg: 0 for leg in self.legs}
        self.done = {leg: False for leg in self.legs}

        print(f"=== INICIANDO CALIBRAÇÃO (QoS Best Effort) ===")
        print(f"Coletando {SAMPLES} amostras... Se travar, verifique se o robô está ativo.")

        for leg in self.legs:
            topic = f'/{leg}_joint_trajectory_controller/controller_state'
            # Usa qos_profile_sensor_data para garantir conexão
            self.create_subscription(
                JointTrajectoryControllerState,
                topic,
                lambda msg, l=leg: self.listener_callback(msg, l),
                qos_profile_sensor_data 
            )

    def listener_callback(self, msg, leg_name):
        if self.done[leg_name]:
            return

        for i in range(3):
            # O array msg.error.positions tem a ordem das juntas no controller
            # Assumindo que a ordem é Roll, Pitch, Elbow (padrão)
            current_error = msg.error.positions[i]
            current_torque = KP_ATUAL * current_error
            self.data[leg_name][i].append(current_torque)

        self.counts[leg_name] += 1
        
        if self.counts[leg_name] % 50 == 0:
            print(f"{leg_name}: {self.counts[leg_name]}/{SAMPLES}...")

        if self.counts[leg_name] >= SAMPLES:
            self.done[leg_name] = True
            self.check_all_done()

    def check_all_done(self):
        if all(self.done.values()):
            print("\n" + "="*50)
            print("       COPIE ISTO PARA O URDF (.xacro)")
            print("="*50 + "\n")
            
            for leg in self.legs:
                print(f"")
                avg_torques = [np.mean(self.data[leg][i]) for i in range(3)]
                joints = [f"{leg}_{s}" for s in self.joint_suffixes]
                
                for i, joint_name in enumerate(joints):
                    torque_val = avg_torques[i]
                    print(f'<joint name="{joint_name}">')
                    print(f'    <param name="ff_torque">{torque_val:.2f}</param>')
                    print(f'</joint>')
                print("")
            
            print("="*50)
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GravityCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()