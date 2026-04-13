find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(BOM_STRIP_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../tools/strip_utf8_bom.py")
set(BOM_SCAN_DIRS
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/../ServoBoardMain"
    "${CMAKE_CURRENT_LIST_DIR}/../libraries/FTServo_Arduino/src"
)

add_custom_target(strip_utf8_bom_sources ALL
    COMMAND ${Python3_EXECUTABLE} "${BOM_STRIP_SCRIPT}" ${BOM_SCAN_DIRS}
    COMMENT "Stripping UTF-8 BOM from project sources"
    VERBATIM
)

set(APP_SRCS
    "app_main.cpp"
    "../ServoBoardMain/SystemTask.cpp"
    "../ServoBoardMain/UpperCommTask.cpp"
    "../ServoBoardMain/CanCommTask.cpp"
    "../ServoBoardMain/AngleSolver.cpp"
    "../ServoBoardMain/CalibrationTask.cpp"
    "../ServoBoardMain/JointCalibrationProfile.cpp"
    "../ServoBoardMain/ServoBusManager.cpp"
    "../ServoBoardMain/pid.c"
    "../libraries/FTServo_Arduino/src/HLSCL.cpp"
    "../libraries/FTServo_Arduino/src/SCS.cpp"
    "../libraries/FTServo_Arduino/src/SCSCL.cpp"
    "../libraries/FTServo_Arduino/src/SCSerial.cpp"
    "../libraries/FTServo_Arduino/src/SMS_STS.cpp"
)

idf_component_register(
    SRCS ${APP_SRCS}
    INCLUDE_DIRS
        "."
        "../ServoBoardMain"
        "../libraries/FTServo_Arduino/src"
)

add_dependencies(${COMPONENT_LIB} strip_utf8_bom_sources)
