## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (WIN32 OR APPLE)
   return()
ENDIF()

execute_process(COMMAND objdump -C -t ${file} OUTPUT_VARIABLE output)
string(REPLACE "\n" ";" output ${output})

foreach (line ${output})
  if ("${line}" MATCHES "O .bss")
    if (NOT "${line}" MATCHES "std::__ioinit" AND          # this is caused by iostream initialization and is likely also ok
        NOT "${line}" MATCHES "\\(\\)::" AND               # this matches a static inside a function which is fine
        NOT "${line}" MATCHES "function_local_static_" AND # static variable inside a function (explicitly named)
        NOT "${line}" MATCHES "__\\$U")                    # ICC generated locks for static variable inside a function
      message(WARNING "\nProblematic global variable in non-SSE code:\n" ${line})
    endif()
  endif()
endforeach()
