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
    yaml_file = 'zed_calibration.yaml'  # Path to the YAML file
    camera_info = '/zed/zed_node/left_raw/camera_info'

    try:
        camera_data = load_camera_info(camera_info, yaml_file)
        print("Camera Info:")
        print(f"Height: {camera_data['height']}")
        print(f"Width: {camera_data['width']}")
        print(f"D: {camera_data['d']}")
        print(f"K: {camera_data['k']}")
        print(f"R: {camera_data['r']}")
        print(f"P: {camera_data['p']}")
    except ValueError as e:
        print(e)


    image_size = (640, 360)

    # 주어진 데이터
    K = np.array([
        [266.2300109863281, 0.0, 321.7174987792969],
        [0.0, 266.2925109863281, 177.7062530517578],
        [0.0, 0.0, 1.0]
    ])
    d = np.array([-1.3494, 2.8427, 0.0002541, 0.0003107, 0.0408976])

    # 입력: 2D 이미지 좌표 (u, v)
    image_point = np.array([[320, 180]], dtype=np.float32)

    # 왜곡 보정
    undistorted_point = cv2.undistortPoints(image_point, K, d, None, K)

    # 정규화된 좌표 계산
    x, y = undistorted_point[0, 0], undistorted_point[0, 1]
    direction_vector = np.array([x, y, 1.0])  # Z = 1로 설정

    # 방향 벡터 정규화 (필요시)
    direction_vector_normalized = direction_vector / np.linalg.norm(direction_vector)

    # 결과 출력
    print("3D Direction Vector (Camera Coordinate):", direction_vector_normalized)



if __name__ == "__main__":
    main()


