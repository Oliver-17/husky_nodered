cmake_minimum_required(VERSION 3.8)
project(husky_nodered)

find_package(ament_cmake REQUIRED)

# 這個套件沒有要編譯的東西，只是把資料檔與 launch 裝到 share/。
# gz/models 裡的 Husky 模型要靠 GZ_SIM_RESOURCE_PATH 找到，launch 會設。
install(DIRECTORY gz launch config docs
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
        scripts/cmd_vel_watchdog.py
        scripts/husky_api_node.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
