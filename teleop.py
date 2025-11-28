import hydra
from openteach.components import TeleOperator
import time
from multiprocessing import Queue

@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'teleop')
def main(configs):
    teleop = TeleOperator(configs)
    processes = teleop.get_processes()
    operator_queue: Queue = teleop.get_operator_command_queue()

    for process in processes:
        process.start()

    try:
        input("[Teleop] Press Enter to send robot to home position. (First try)")
        operator_queue.put("HOME")

        input("[Teleop] Press Enter to send robot to home position. (Second try)")
        operator_queue.put("HOME")

        input("[Teleop] Press Enter to send robot to home position. (Third try)")
        operator_queue.put("HOME")
    except KeyboardInterrupt:
        print("\n[Teleop] KeyboardInterrupt detected. Exiting...")
    finally:
        for process in processes:
            process.join()

    print("[Teleop] All processes have been joined. Exiting program.")

if __name__ == '__main__':
    main()
