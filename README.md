# 도커를 활용한 자율주행 로봇 시뮬레이션 자료

* Dockerfile 설치

        [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)

* Dockerfile 빌드

        sudo docker build --progress=plain -t nav2:humble ./

* Dockerfile 실행 & 접속

        sudo docker run -it -d --network=host --privileged --env="DISPLAY=$DISPLAY" --volume="${XAUTHORITY}:/root/.Xauthority" nav2:humble

        sudo docker ps

        sudo docker exec -it (container id) bash

* 시뮬레이션 실행

        ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

* Cartographer

        ros2 launch turtlebot3_cartographer cartographer.launch.py

        ros2 run nav2_map_server map_saver_cli --free 0.196 -f ./my_map

        ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url : /my_map.yaml}"

* Ros2 패키지 만들기

        mkdir -p /ros2_ws/src && cd /ros2_ws/src

        ros2 pkg create --build-type ament_python examples

        gedit /ros2_ws/src/examples/examples/cmd_vel_example.py

        gedit /ros2_ws/src/examples/examples/scan_example.py

        gedit /ros2_ws/src/examples/examples/image_example.py
        
        

* entry_points 설정

        gedit /ros2_ws/src/examples/setup.py

        entry_points={
                'console_scripts': [
                        'cmd_vel_example = examples.cmd_vel_example:main',
                        'scan_example = examples.scan_example:main',
                        'image_example = examples.image_example:main',
                ],
        },

* Ros2 패키지 빌드

        cd /ros2_ws

        colcon build --symlink-install

* Ros2 패키지 실행

        source /ros2_ws/install/local_setup.bash

        ros2 run examples cmd_vel_example


        ros2 run examples scan_example


        ros2 run examples image_example

* Param 조회 & 수정

        # Cmd Vel Scale 조회 & 수정
        ros2 param get /new_cmd_publisher linear_scale
        ros2 param get /new_cmd_publisher angular_scale
        ros2 param set /new_cmd_publisher linear_scale 0.8
        ros2 param set /new_cmd_publisher angular_scale 1.5

        # Obstacle Detect Range 조회 & 수정
        ros2 param get /scan_example obstacle_detect_range
        ros2 param set /scan_example obstacle_detect_range 0.7
