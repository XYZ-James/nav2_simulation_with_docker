# 도커를 활용한 자율주행 로봇 시뮬레이션 자료

* Dockerfile 설치

       https://docs.docker.com/engine/install/ubuntu/

* Dockerfile 빌드

        sudo docker build --progress=plain -t nav2:humble ./

* Docker 관리

        sudo docker run -it -d --network=host --privileged --env="DISPLAY=$DISPLAY" --volume="${XAUTHORITY}:/root/.Xauthority" --volume="/home/james/docker_ws:/docker_ws" --name tb3 nav2:humble

        sudo docker ps -a

        sudo docker exec -it tb3 bash

        docker commit tb3 updated_nav2

        docker stop tb3

        docker rm tb3

* 시뮬레이션 실행

        ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

        ros2 run teleop_twist_keyboard teleop_twist_keyboard
  

* URDF & SDF 수정
  
        URDF 수정
  
        cd /opt/ros/humble/share/nav2_bringup/urdf
        cp turtlebot3_waffle.urdf turtlebot3_waffle_backup.urdf
        gedit turtlebot3_waffle.urdf

        SDF 수정

        cd /opt/ros/humble/share/nav2_bringup/worlds
        cp waffle.model waffle_backup.model
        gedit waffle.model

* Cartographer

        ros2 launch turtlebot3_cartographer cartographer.launch.py

        ros2 run nav2_map_server map_saver_cli --free 0.196 -f /docker_ws/my_map

        ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url : /docker_ws/my_map.yaml}"

* Ros2 패키지 만들기

        mkdir -p /docker_ws/src && cd /docker_ws/src

        ros2 pkg create --build-type ament_python examples

        cd /docker_ws/src/examples/examples && gedit cmd_vel_example.py

        cd /docker_ws/src/examples/examples && gedit scan_example.py

        cd /docker_ws/src/examples/examples && gedit image_example.py
        
        

* entry_points 설정

        gedit /docker_ws/src/examples/setup.py

        entry_points={
                'console_scripts': [
                        'cmd_vel_example = examples.cmd_vel_example:main',
                        'scan_example = examples.scan_example:main',
                        'image_example = examples.image_example:main',
                ],
        },

* Ros2 패키지 빌드

        cd /docker_ws

        colcon build --symlink-install

* Ros2 패키지 실행

        source /docker_ws/install/local_setup.bash

        ros2 run examples cmd_vel_example


        ros2 run examples scan_example


        ros2 run examples image_example

* Param 조회 & 수정

        # Cmd Vel Scale 조회 & 수정
        ros2 param get /cmd_vel_example linear_scale
        ros2 param get /cmd_vel_example angular_scale
        ros2 param set /cmd_vel_example linear_scale 0.8
        ros2 param set /cmd_vel_example angular_scale 1.5
  
        # Obstacle Detect Range 조회 & 수정
        ros2 param get /scan_example obstacle_detect_range
        ros2 param set /scan_example obstacle_detect_range 0.7
