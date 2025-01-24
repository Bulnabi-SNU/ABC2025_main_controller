import numpy as np
import cv2
import yaml



def load_camera_info(camera_info, yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)

    # Find the specified camera_info section
    if camera_info in data:
        info = data[camera_info]
        
        # Extract the relevant fields
        height = info['height']
        width = info['width']
        d = info['d']
        k = info['k']
        r = info['r']
        p = info['p']

        return {
            'height': height,
            'width': width,
            'd': d,
            'k': k,
            'r': r,
            'p': p
        }
    
    else:
        raise ValueError(f"Camera info '{camera_info}' not found in the file.")



# Example usage
def main():
    yaml_file = 'src/camera_calibration/zed_calibration_formatted.yaml'  # Path to the YAML file

    with open(yaml_file, 'r') as file:
        camera_data = yaml.full_load(file)
    
    # 데이터 변환 후 각 변수에 저장
    height = float(camera_data['height'])  # Height(float)
    width = float(camera_data['width'])   # Width(float)
    d = np.array(camera_data['d'], dtype=np.float32)  # d(np.array)
    k = np.array(camera_data['k'], dtype=np.float32)  # k(np.array)
    r = np.array(camera_data['r'], dtype=np.float32)  # r(np.array)
    p = np.array(camera_data['p'], dtype=np.float32)  # p(np.array)

    # 결과 출력
    print("Height (float):", height)
    print("Width (float):", width)
    print("d (np.array):", d)
    print("k (np.array):\n", k)
    print("r (np.array):\n", r)
    print("p (np.array):\n", p)

    # p_inverse = np.linalg.inv(p)
    k_inverse = np.linalg.inv(k)

    u = 180
    v = 280
    s = 1
    target2d = s * np.array([u, v, 1], dtype=np.float32)

    target3d = np.dot(k_inverse, target2d)
    delta_yaw = np.arctan(target3d[0] / target3d[2])
    delta_pitch = np.arctan(target3d[1] / target3d[2])

    target_position_15m = target3d/np.linalg.norm(target3d) * 15

    print("\n\ntarget3d:", target3d)
    print("delta_yaw:", delta_yaw)
    print("delta_pitch:", delta_pitch)
    print("target_position_15m:", target_position_15m)


if __name__ == "__main__":
    main()


