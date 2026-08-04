if(NOT DEFINED COMMAND_PATH OR NOT DEFINED EXPECTED_FILE)
    message(FATAL_ERROR "COMMAND_PATH and EXPECTED_FILE are required")
endif()

execute_process(
    COMMAND "${COMMAND_PATH}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    RESULT_VARIABLE command_result
    OUTPUT_VARIABLE actual_output
    ERROR_VARIABLE command_error)

if(NOT command_result EQUAL 0)
    message(
        FATAL_ERROR
        "command failed with ${command_result}:\n${command_error}")
endif()

file(READ "${EXPECTED_FILE}" expected_output)
if(NOT actual_output STREQUAL expected_output)
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/compatibility-actual.txt" "${actual_output}")
    message(
        FATAL_ERROR
        "compatibility output differs from ${EXPECTED_FILE}; "
        "actual output written to compatibility-actual.txt")
endif()
