# usage: cmake -DSIM=<exe> -DARGS="a;b;c" -P expect_exit2.cmake
# 引数の拒否(終了コード 2)を確かめる。
execute_process(COMMAND ${SIM} ${ARGS} RESULT_VARIABLE rc OUTPUT_QUIET ERROR_QUIET)
if(NOT rc EQUAL 2)
  message(FATAL_ERROR "expected exit code 2, got: ${rc} (args: ${ARGS})")
endif()
