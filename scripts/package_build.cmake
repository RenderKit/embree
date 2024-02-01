
message("Fixing package names")

message("PACKAGE_BASENAME = ${PACKAGE_BASENAME}")
message("PACKAGE_EXT      = ${PACKAGE_EXT}")

file(RENAME "${PACKAGE_BASENAME}-embree.${PACKAGE_EXT}" "${PACKAGE_BASENAME}.${PACKAGE_EXT}")
file(RENAME "${PACKAGE_BASENAME}-embree-testing.${PACKAGE_EXT}" "${PACKAGE_BASENAME}-testing.${PACKAGE_EXT}")
if (EXISTS "${PACKAGE_BASENAME}-Unspecified.${PACKAGE_EXT}")
    file(REMOVE "${PACKAGE_BASENAME}-Unspecified.${PACKAGE_EXT}")
endif()

message("Created packages:")
message("${PACKAGE_BASENAME}.${PACKAGE_EXT}")
message("${PACKAGE_BASENAME}-testing.${PACKAGE_EXT}")