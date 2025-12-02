import h5py
import pickle
import zmq
import numpy as np
import time
import hydra
from openteach.constants import DEPLOY_FREQ

@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'replay')
def main(configs):
    DEMO_PATH = configs.demo_path
    HOST_ADDRESS = configs.host_address
    DEPLOYMENT_PORT = configs.deployment_port
    
    ur_actions = load_ur_demonstration(DEMO_PATH)
    
    if ur_actions:
        send_actions_to_deployer(ur_actions, HOST_ADDRESS, DEPLOYMENT_PORT)


def load_ur_demonstration(demo_path):
    """
    Load cartesian and gripper states for UR robot from h5 files located in demo_path.
    """
    
    # Cartesian states: positions (x,y,z) + orientations (roll, pitch, yaw)
    with h5py.File(f'{demo_path}/ur_cartesian_states.h5', 'r') as f:
        positions = f['positions'][:]
        rpys = f['orientations'][:]
        
        cartesian_actions = np.hstack((positions, rpys)) 

    # Gripper states: normalized positions
    with h5py.File(f'{demo_path}/ur_gripper_states.h5', 'r') as f:
        gripper_actions = f['normalized_positions'][:] 

    # Ensure both action sequences have the same length
    min_length = min(len(cartesian_actions), len(gripper_actions))
    
    actions = []
    for i in range(min_length):
        action_dict = {
            'ur': cartesian_actions[i].tolist() + [1 - gripper_actions[i][0]], # Append gripper state to UR action
        }
        actions.append(action_dict)

    print(f"Loaded {len(actions)} action steps.")
    return actions


def send_actions_to_deployer(actions, host_address, deployment_port):
    """
    Send loaded robot actions to the DeployServer.
    """
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f'tcp://{host_address}:{deployment_port}')
    
    print(f"Connected to DeployServer: tcp://{host_address}:{deployment_port}")
    
    timer_interval = 1 / DEPLOY_FREQ

    for step, action in enumerate(actions):
        start_time = time.time()
        
        # Add gripper action to UR robot action dictionary (for DeployServer to process)
        action_to_send = {'ur': action['ur']}
        
        socket.send(pickle.dumps(action_to_send, protocol=-1))
        
        try:
            response_data = socket.recv() 
            response = pickle.loads(response_data)
            
            if isinstance(response, str) and response == "Command failed!":
                print(f"❗ Step {step}: Robot control failed. Sequence aborted.")
                break

        except Exception as e:
            print(f"Step {step}: Error processing response: {e}")
            break
        
        # Maintain control cycle timing
        elapsed_time = time.time() - start_time
        sleep_duration = timer_interval - elapsed_time
        if sleep_duration > 0:
            time.sleep(sleep_duration)
        
    socket.close()
    context.term()
    print("Robot control sequence completed.")

if __name__ == '__main__':
    main()