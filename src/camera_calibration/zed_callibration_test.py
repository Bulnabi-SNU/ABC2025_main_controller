import cv2
import pyzed.sl as sl

def main():
    # ZED 카메라 초기화
    zed = sl.Camera()

    # 초기 설정
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # 해상도 설정
    init_params.depth_mode = sl.DEPTH_MODE.NONE  # 깊이 정보 비활성화

    # 카메라 열기
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("카메라를 열 수 없습니다.")
        exit(1)

    # 이미지 캡처 객체
    left_image = sl.Mat()
    right_image = sl.Mat()

    print("ZED 카메라 스트리밍 시작...")
    try:
        while True:
            # 새로운 프레임 캡처
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                # 좌우 이미지 가져오기
                zed.retrieve_image(left_image, sl.VIEW.LEFT)
                zed.retrieve_image(right_image, sl.VIEW.RIGHT)

                # 이미지를 numpy 배열로 변환
                left_frame = left_image.get_data()
                right_frame = right_image.get_data()

                # OpenCV로 표시
                cv2.imshow("Left Image", left_frame)
                cv2.imshow("Right Image", right_frame)

                # 'q' 키를 누르면 종료
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        print("스트리밍 중지...")
    finally:
        # 리소스 해제
        cv2.destroyAllWindows()
        zed.close()

if __name__ == "__main__":
    main()