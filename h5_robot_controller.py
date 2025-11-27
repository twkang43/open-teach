import h5py
import pickle
import zmq
import numpy as np
import time

# network.yaml에서 설정된 값
HOST_ADDRESS = '192.168.1.90'
DEPLOYMENT_PORT = 10000 
DEPLOY_FREQ = 60 # DeployServer의 제어 주기를 20Hz로 가정

# H5 파일에서 필요한 데이터를 로드하는 함수
def load_ur_demonstration(demo_path):
    """
    UR 로봇 시연 파일에서 카르테시안 위치/방향과 그리퍼 상태를 로드합니다.
    """
    
    # 카르테시안 상태 로드 (위치 3D + 방향 3D)
    with h5py.File(f'{demo_path}/ur_cartesian_states.h5', 'r') as f:
        # [cite_start]H5 파일 메타데이터(ur_cartesian_states.h5)에서 데이터셋 'positions'와 'rpys'를 확인 [cite: 181]
        positions = f['positions'][:]
        rpys = f['orientations'][:]
        
        # 6D 카르테시안 액션 (x, y, z, roll, pitch, yaw) 결합
        cartesian_actions = np.hstack((positions, rpys)) 

    # 그리퍼 상태 로드
    with h5py.File(f'{demo_path}/ur_gripper_states.h5', 'r') as f:
        # [cite_start]H5 파일 메타데이터(ur_gripper_states.h5)에서 데이터셋 'normalized_positions'를 확인 [cite: 612]
        # 그리퍼 액션은 일반적으로 1차원 값입니다.
        gripper_actions = f['normalized_positions'][:] 

    # 두 액션 시퀀스의 길이가 일치하는지 확인 (가장 짧은 길이에 맞춤)
    min_length = min(len(cartesian_actions), len(gripper_actions))
    
    actions = []
    for i in range(min_length):
        # DeployServer의 _perform_robot_action이 기대하는 딕셔너리 포맷
        # ur: 6D 카르테시안 액션, gripper_state: 1D 그리퍼 액션
        action_dict = {
            'ur': cartesian_actions[i], # 6D 벡터
            'gripper_state': gripper_actions[i][0] # 1D 스칼라 값 (혹은 1D 벡터)
        }
        actions.append(action_dict)

    print(f"총 {len(actions)}개의 액션 스텝을 로드했습니다.")
    return actions


def send_actions_to_deployer(actions):
    """로드된 로봇 액션을 DeployServer로 전송합니다."""
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f'tcp://{HOST_ADDRESS}:{DEPLOYMENT_PORT}')
    
    print(f"DeployServer에 연결: tcp://{HOST_ADDRESS}:{DEPLOYMENT_PORT}")
    
    timer_interval = 1 / DEPLOY_FREQ

    for step, action in enumerate(actions):
        start_time = time.time()
        
        # 그리퍼 액션을 UR 로봇 액션 딕셔너리에 추가 (DeployServer가 처리할 수 있도록)
        action_to_send = {
            'ur': action['ur'],
            # 'gripper_state': action['gripper_state']
        }
        
        socket.send(pickle.dumps(action_to_send, protocol=-1))
        
        try:
            # 서버 응답 수신
            response_data = socket.recv() 
            response = pickle.loads(response_data)
            
            if isinstance(response, str) and response == "Command failed!":
                print(f"❗ Step {step}: 로봇 제어 실패. 시퀀스 중단.")
                break

        except Exception as e:
            print(f"Step {step}: 응답 처리 중 오류 발생: {e}")
            break
        
        # 제어 주기 맞추기
        elapsed_time = time.time() - start_time
        sleep_duration = timer_interval - elapsed_time
        if sleep_duration > 0:
            time.sleep(sleep_duration)
        
    socket.close()
    context.term()
    print("로봇 제어 시퀀스 완료.")

if __name__ == '__main__':
    # 시연 파일 경로 (ur_cartesian_states.h5 등이 포함된 디렉토리)
    DEMO_PATH = 'extracted_data/put_lemon_in_bowl/demonstration_0' # [cite: 198, 378, 614]
    
    ur_actions = load_ur_demonstration(DEMO_PATH)
    
    if ur_actions:
        send_actions_to_deployer(ur_actions)