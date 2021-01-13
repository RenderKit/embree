## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

IF (WIN32 OR APPLE)
   return()
ENDIF()

execute_process(COMMAND objdump -d ${file} OUTPUT_VARIABLE output)
string(REPLACE "\n" ";" output ${output})

SET(lastfunc "")
foreach (line ${output})
  if ("${line}" MATCHES "^.*<([^>]*)>:$")
    SET(lastfunc ${CMAKE_MATCH_1})
  endif()
  if ("${line}" MATCHES ".*sub[ ]+[$]([^,]*),%rsp.*")
    set(bytes ${CMAKE_MATCH_1})
    if ("${bytes}" GREATER "4096")
      if ("${lastfunc}" MATCHES ".*recurse.*")
        message(WARNING "Large stack space requirement: ${lastfunc} size: ${bytes}")
      endif()
    endif()
  endif()
endforeach()