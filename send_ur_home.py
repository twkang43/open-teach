import pickle
import zmq
import hydra

@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'replay')
def main(configs):
    host_address = configs.host_address
    deployment_port = configs.deployment_port

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(f'tcp://{host_address}:{deployment_port}')
    
    print(f"Connected to DeployServer: tcp://{host_address}:{deployment_port}")

    action = {'ur_home': True}
    socket.send(pickle.dumps(action, protocol=-1))
    try:
        response_data = socket.recv() 
        response = pickle.loads(response_data)
        
        if isinstance(response, str) and response == "Command failed!":
            print("Sending UR to home failed. Sequence aborted.")

    except Exception as e:
        print(f"Error processing response: {e}")

    socket.close()
    context.term()
    print("Sending UR to home completed.")

if __name__ == '__main__':
    main()