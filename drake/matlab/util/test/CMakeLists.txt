drake_add_matlab_test(NAME drake/matlab/util:call_matlab_client_test
  COMMAND "call_matlab_client('${CMAKE_CURRENT_SOURCE_DIR}/call_matlab_test.rpc')")
