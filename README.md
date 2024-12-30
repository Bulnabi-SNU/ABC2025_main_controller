# zed_camera_nodes
ROS2 nodes for dynamic stabilization project (shit balloon)

# Installation

1. 해당 repo fork  (Click Create Fork)

2. workspace 만들기

   ```
   cd
   mkdir zed_ws
   cd zed_ws
   ```

3. 본인 repository에서 fork된 repo clone

   ```
   git clone https://github.com/[your_github_id]/zed_camera_nodes.git
   ```

4. src 폴더로 이름 변경

   ```
   mv zed_camera_nodes src
   ```

5. upstream 등록

   ```
   cd src
   git remote add upstream https://github.com/Bulnabi-SNU/zed_camera_nodes.git
   ```

7. git 연결 확인

   ```
   git remote -v
   ```

   (정상적인 출력 결과)

   ```
    origin	https://github.com/[your_github_id]/zed_camera_nodes.git (fetch)
    origin	https://github.com/[your_github_id]/zed_camera_nodes.git (push)
    upstream	https://github.com/Bulnabi-SNU/zed_camera_nodes.git (fetch)
    upstream	https://github.com/Bulnabi-SNU/zed_camera_nodes.git (push)
   ```
