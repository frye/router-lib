if(NOT DEFINED COMMAND_PATH OR
   NOT DEFINED COMMAND_ARGUMENTS OR
   NOT DEFINED EXPECTED_OUTPUT)
    message(FATAL_ERROR "command failure test is missing required configuration")
endif()

execute_process(
    COMMAND "${COMMAND_PATH}" ${COMMAND_ARGUMENTS}
    RESULT_VARIABLE command_result
    OUTPUT_VARIABLE command_stdout
    ERROR_VARIABLE command_stderr
)

if(command_result EQUAL 0)
    message(FATAL_ERROR "command unexpectedly succeeded")
endif()

string(CONCAT command_output "${command_stdout}" "${command_stderr}")
if(NOT command_output MATCHES "${EXPECTED_OUTPUT}")
    message(
        FATAL_ERROR
        "command output did not match '${EXPECTED_OUTPUT}':\n${command_output}")
endif()
