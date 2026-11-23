cmake_minimum_required(VERSION 3.16)
project(social_nav_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

file(GLOB CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(social_nav_core STATIC ${CORE_SOURCES})
add_library(social_nav_core::social_nav_core ALIAS social_nav_core)
target_include_directories(social_nav_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(social_nav_core PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(social_nav_core PRIVATE -Wall -Wextra)
endif()

# Tests: test/test_*.cpp -> one executable per file
set_property(GLOBAL PROPERTY CTEST_TARGETS_ADDED 1)
include(CTest)
if(BUILD_TESTING)
file(GLOB CORE_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_*.cpp)
list(FILTER CORE_TESTS EXCLUDE REGEX "/test_sim_[^/]*$")   # sim 用テストは下で social_nav_sim_lib に link する
foreach(test_src ${CORE_TESTS})
  get_filename_component(test_name ${test_src} NAME_WE)
  add_executable(${test_name} ${test_src})
  target_link_libraries(${test_name} PRIVATE social_nav_core)
  target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${test_name} PRIVATE -Wall -Wextra)
  endif()
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
endif()

# Simulator (ROS 非依存。install 対象外)
file(GLOB SIM_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/sim/*.cpp)
list(FILTER SIM_SOURCES EXCLUDE REGEX "/main\\.cpp$")
add_library(social_nav_sim_lib STATIC ${SIM_SOURCES})
target_include_directories(social_nav_sim_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(social_nav_sim_lib PUBLIC social_nav_core)
add_executable(social_nav_sim ${CMAKE_CURRENT_SOURCE_DIR}/sim/main.cpp)
target_link_libraries(social_nav_sim PRIVATE social_nav_sim_lib)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(social_nav_sim_lib PRIVATE -Wall -Wextra)
  target_compile_options(social_nav_sim PRIVATE -Wall -Wextra)
endif()
# 煙テスト(1 シード)。合否判定ではない。判定は M1a(走行中、または停止後 1.0 s 未満の衝突)の回帰の上限で、
# 1.0 s 以上停止中の被接触(M1b)は内訳で報告する(design 6.3)。crowd 以外は --check-moving(シード 1 で M1a = 0)、crowd は下の --check-moving-max
foreach(scenario corridor_oncoming crossing overtake follow static_group)
  add_test(NAME sim_smoke_${scenario}
           COMMAND social_nav_sim --scenario ${scenario} --seeds 1 --check-moving)
endforeach()
# crowd は回帰の上限として --check-moving-max で置く(シード 1 の sfm2 の値は 0)。判定 21-120 の social は 159 以下(sfm2。docs/sim_results.md, design.md 決定 3)
add_test(NAME sim_smoke_crowd
         COMMAND social_nav_sim --scenario crowd --seeds 1 --check-moving-max 0)
# legacy(--ped-model legacy)の完走 smoke。M1 の閾値は置かない(既定の sfm2 は上の smoke が見る)
add_test(NAME sim_smoke_legacy_corridor_oncoming
         COMMAND social_nav_sim --scenario corridor_oncoming --seeds 1 --ped-model legacy --check-moving)
# 後方からの人流(記録のみ)。完走だけ見る(M1 の閾値は置かない)。--scenario all には入らない
foreach(scenario rear_follow rear_overtake rear_overtake_flow)
  add_test(NAME sim_smoke_${scenario}
           COMMAND social_nav_sim --scenario ${scenario} --seeds 1)
endforeach()
# 屋内 3 本(記録のみ)。完走だけ見る。--scenario all には入らない
foreach(scenario door_oncoming side_door group_oncoming)
  add_test(NAME sim_smoke_${scenario}
           COMMAND social_nav_sim --scenario ${scenario} --seeds 1)
endforeach()
# 同じ seed の 2 回の実行で trace がバイト一致(side_door、5 s)
add_test(NAME sim_repro_indoor_trace
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> -DOUT=${CMAKE_CURRENT_BINARY_DIR}/repro_indoor_trace
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/test/repro_indoor_trace.cmake)
add_test(NAME sim_args_trace_indoor_group
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--scenario;indoor;--seeds;1;--trace;x.txt" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
# 見えない人(記録のみ)。完走だけ見る。--scenario all には入らない
add_test(NAME sim_smoke_blind_corner
         COMMAND social_nav_sim --scenario blind_corner --seeds 1 --occlusion --blind-csv ${CMAKE_CURRENT_BINARY_DIR}/smoke_blind.csv)
add_test(NAME sim_args_trace_blind_group
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--scenario;blind;--seeds;1;--trace;x.txt" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
# 壁なしシナリオで --occlusion の on / off が一致、--blind-csv の列数、遮蔽の効き
add_test(NAME sim_occlusion_check
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> -DOUT=${CMAKE_CURRENT_BINARY_DIR}/occlusion_check
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/test/occlusion_check.cmake)
# 急に止まる・振り返る人(記録のみ)。完走だけ見る。--scenario all には入らない
foreach(scenario sudden_stop sudden_turn sudden_reverse)
  add_test(NAME sim_smoke_${scenario}
           COMMAND social_nav_sim --scenario ${scenario} --seeds 1 --event-csv ${CMAKE_CURRENT_BINARY_DIR}/smoke_${scenario}_event.csv)
endforeach()
add_test(NAME sim_args_trace_sudden_group
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--scenario;sudden;--seeds;1;--trace;x.txt" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
# エレベーター前・出口の流れ(候補 6。記録のみ)。完走だけ見る(sfm2 / ttc / orca)。--scenario all には入らない
foreach(model sfm2 ttc orca)
  add_test(NAME sim_smoke_elevator_flow_${model}
           COMMAND social_nav_sim --scenario elevator_flow --seeds 1 --ped-model ${model})
endforeach()
add_test(NAME sim_args_trace_elevator_group
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--scenario;elevator;--seeds;1;--trace;x.txt" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
# 速さの違う人(候補 8。記録のみ・既定 off)。完走だけ見る(on の 3 モデル。対象の crowd)。既定の --ped-speed-mix は off(無指定と同じ出力)
foreach(model sfm2 ttc orca)
  add_test(NAME sim_smoke_speed_mix_${model}
           COMMAND social_nav_sim --scenario crowd --seeds 1 --ped-model ${model} --ped-speed-mix on --sim-time 20)
endforeach()
add_test(NAME sim_args_ped_speed_mix_invalid
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--ped-speed-mix;maybe" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
add_test(NAME sim_speed_mix_defaults_unchanged
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> -DOUT=${CMAKE_CURRENT_BINARY_DIR}/speed_mix_defaults
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/test/speed_mix_defaults.cmake)
# 同じ seed の 2 回の実行で trace がバイト一致(rear_overtake_flow、5 s)
add_test(NAME sim_repro_rear_trace
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> -DOUT=${CMAKE_CURRENT_BINARY_DIR}/repro_rear_trace
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/test/repro_rear_trace.cmake)
add_test(NAME sim_args_trace_rear_group
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--scenario;rear;--seeds;1;--trace;x.txt" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
# 再現性: sfm2 の crowd を seed 1 で短時間(5 s)2 回走らせ、trace がバイト一致すること(同じ環境内の比較)
add_test(NAME sim_repro_sfm2_trace
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> -DOUT=${CMAKE_CURRENT_BINARY_DIR}/repro_trace
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/test/repro_trace.cmake)
# 引数の拒否(終了コード 2)
add_test(NAME sim_args_ped_model_invalid
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--ped-model;foo" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
add_test(NAME sim_args_ped_variants_invalid
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--ped-variants;maybe" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
add_test(NAME sim_args_check_moving_max_negative
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--check-moving-max;-1" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
add_test(NAME sim_args_check_moving_max_nan
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--check-moving-max;abc" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
add_test(NAME sim_args_tracker_q_nan
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--use-tracker;--tracker-q;abc" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
add_test(NAME sim_args_tracker_q_nonpositive
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--use-tracker;--tracker-q;0" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
add_test(NAME sim_args_tracker_q_without_use_tracker
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--tracker-q;2.0" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
foreach(case "prox_sigma_zero;--prox-sigma;0" "prox_sigma_nan;--prox-sigma;abc" "prox_sigma_inf;--prox-sigma;inf" "prox_sigma_neg;--prox-sigma;-0.5"
             "rear_ratio_zero;--rear-ratio;0" "rear_ratio_nan;--rear-ratio;abc" "rear_ratio_trailing;--rear-ratio;0.8x")
  list(GET case 0 cname)
  list(SUBLIST case 1 2 cargs)
  add_test(NAME sim_args_${cname}
           COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=${cargs}" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
endforeach()
# --prox-sigma / --rear-ratio: 既定値の明示は無指定と同じ出力、値を変えると出力が変わる
add_test(NAME sim_prox_defaults_unchanged
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> -DOUT=${CMAKE_CURRENT_BINARY_DIR}/prox_defaults
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/test/prox_defaults.cmake)
# --safety-mode / --diag-ped-robot-vel: 不正値は終了コード 2、既定値の明示は無指定と同じ出力
foreach(case "safety_mode_invalid;--safety-mode;foo" "safety_mode_uppercase;--safety-mode;CM" "diag_ped_vel_invalid;--diag-ped-robot-vel;both")
  list(GET case 0 cname)
  list(SUBLIST case 1 2 cargs)
  add_test(NAME sim_args_${cname}
           COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=${cargs}" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
endforeach()
add_test(NAME sim_args_safety_mode_missing
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> "-DARGS=--safety-mode" -P ${CMAKE_CURRENT_SOURCE_DIR}/test/expect_exit2.cmake)
add_test(NAME sim_safety_defaults_unchanged
         COMMAND ${CMAKE_COMMAND} -DSIM=$<TARGET_FILE:social_nav_sim> -DOUT=${CMAKE_CURRENT_BINARY_DIR}/safety_defaults
                 -P ${CMAKE_CURRENT_SOURCE_DIR}/test/safety_defaults.cmake)
# 歩行者モデルの計測(ロボットなし)。sim/ の GLOB の外(probe/)に置く。ctest には入れず tools/run_ped_probe.sh で実行する
add_executable(ped_probe ${CMAKE_CURRENT_SOURCE_DIR}/probe/ped_probe.cpp)
target_link_libraries(ped_probe PRIVATE social_nav_sim_lib)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(ped_probe PRIVATE -Wall -Wextra)
endif()
# 接触直前の止まり方の診断(trace から候補の費用の内訳と最接近すきまを出す)。probe/ に置き、tools/run_cand_probe.sh で実行する
add_executable(cand_probe ${CMAKE_CURRENT_SOURCE_DIR}/probe/cand_probe.cpp)
target_link_libraries(cand_probe PRIVATE social_nav_sim_lib)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(cand_probe PRIVATE -Wall -Wextra)
endif()
file(GLOB SIM_TESTS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sim_*.cpp)
foreach(test_src ${SIM_TESTS})
  get_filename_component(test_name ${test_src} NAME_WE)
  add_executable(${test_name} ${test_src})
  target_link_libraries(${test_name} PRIVATE social_nav_sim_lib)
  target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${test_name} PRIVATE -Wall -Wextra)
  endif()
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

# Install
install(TARGETS social_nav_core EXPORT social_nav_coreTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT social_nav_coreTargets
  FILE social_nav_coreTargets.cmake
  NAMESPACE social_nav_core::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/social_nav_core)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/social_nav_coreConfig.cmake
"include(\"\${CMAKE_CURRENT_LIST_DIR}/social_nav_coreTargets.cmake\")\n")
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/social_nav_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/social_nav_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/social_nav_coreConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/social_nav_core)
