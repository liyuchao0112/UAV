# =========================================================================
# 1. 显式定义当前编译的板子类型（在这里切换 GIMBAL_BOARD 或 CHASSIS_BOARD）
# =========================================================================
set(BOARD "GIMBAL_BOARD")
#set(BOARD "CHASSIS_BOARD")

# =========================================================================
# 2. 声明Sentry目标文件库（OBJECT库，仅编译目标文件不链接，保留原工程逻辑）
# =========================================================================
add_library(Sentry OBJECT)

# 将 CMake 变量的值作为宏传递给 C/C++ 编译器代码
target_compile_definitions(Sentry PRIVATE
        BOARD=${BOARD}
        GIMBAL_BOARD=1
        CHASSIS_BOARD=2
)

# ===================== 所有源文件（严格匹配文件名/大小写/路径） =====================
if (BOARD STREQUAL "GIMBAL_BOARD")
    target_sources(Sentry
            PRIVATE
            # === Application ===

            # === Gimbal 模块 ===
        
            # === Booster 模块 ===

            # === Communication ===

            pyro_mission_planer.cpp
            pyro_init_thread.cpp
        )
    
    # 针对 Gimbal 的特殊头文件包含
    target_include_directories(Sentry
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/Modules/Booster
            ${CMAKE_CURRENT_SOURCE_DIR}/Modules/Gimbal
            ${CMAKE_CURRENT_SOURCE_DIR}/Modules/Communication/Gimbal_board
            ${CMAKE_CURRENT_SOURCE_DIR}
        )
    target_compile_definitions(PYRo PUBLIC
            DR16_UART=PYRO_UART5
            VT03_UART=PYRO_UART1
            AUTOAIM_UART=PYRO_UART7
        )

elseif (BOARD STREQUAL "CHASSIS_BOARD")
    target_sources(Sentry
            PRIVATE
            # === Application ===
            Application/Chassis_Board/pyro_chassis_app.cpp

            # === Chassis 模块 ===

            pyro_mission_planer.cpp
            pyro_init_thread.cpp
        )

    # 针对 Chassis 的特殊头文件包含
    target_include_directories(Sentry
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/Modules/Chassis
            ${CMAKE_CURRENT_SOURCE_DIR}
        )
    target_compile_definitions(PYRo PUBLIC
            REFEREE_UART=PYRO_UART1
            SUPERCAP_UART=PYRO_UART7
        )

endif()

# ===================== 头文件包含路径（覆盖所有模块根目录） =====================
target_include_directories(Sentry
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Modules/Booster
        ${CMAKE_CURRENT_SOURCE_DIR}/Modules/Chassis
        ${CMAKE_CURRENT_SOURCE_DIR}/Modules/Gimbal
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

# ===================== 链接依赖库（保留原工程依赖，可按需增删） =====================
target_link_libraries(Sentry
        PUBLIC
        stm32cubemx
        dsppp_lib
        PYRo
)
