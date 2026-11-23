# usage: cmake -DSIM=<exe> -DOUT=<dir> -P repro_trace.cmake
# sfm2 の crowd を seed 1 で 5 s 走らせる処理を 2 回行い、trace がバイト一致することを確かめる。
file(MAKE_DIRECTORY ${OUT})
foreach(i 1 2)
  execute_process(COMMAND ${SIM} --scenario crowd --seeds 1 --seed-start 1 --ped-model sfm2 --sim-time 5 --trace ${OUT}/trace_${i}.txt
                  RESULT_VARIABLE rc OUTPUT_QUIET ERROR_QUIET)
  if(NOT rc EQUAL 0)
    message(FATAL_ERROR "sim run ${i} failed: ${rc}")
  endif()
endforeach()
file(SIZE ${OUT}/trace_1.txt sz)
if(sz LESS 1000)
  message(FATAL_ERROR "trace too small: ${sz}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} -E compare_files ${OUT}/trace_1.txt ${OUT}/trace_2.txt RESULT_VARIABLE diff)
if(NOT diff EQUAL 0)
  message(FATAL_ERROR "sfm2 trace differs between two runs (seed 1)")
endif()
