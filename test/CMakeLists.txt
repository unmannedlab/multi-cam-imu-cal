##--------------------------------------------------------------------------------------------------------------------##
##                                                                                                                    ##
##                                                    xCalibrate:                                                     ##
##                                                                                                                    ##
##                                       Kalman Filter-Based Sensor Calibration                                       ##
##                                                                                                                    ##
##                                          Copyright (C) 2022 Jacob Hartzer                                          ##
##                                                                                                                    ##
## This program is free software: you can redistribute it and/or modify it under the terms of the                     ##
## GNU General Public License as published by the Free Software Foundation, either version 3 of the License,          ##
## or (at your option) any later version.                                                                             ##
##                                                                                                                    ##
## This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;                          ##
## without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                          ##
## See the GNU General Public License for more details.                                                               ##
##                                                                                                                    ##
## You should have received a copy of the GNU General Public License along with this program.                         ##
## If not, see <https://www.gnu.org/licenses/>.                                                                       ##
##                                                                                                                    ##
##--------------------------------------------------------------------------------------------------------------------##

# Reset ALL_TEST_FILES variable
set(ALL_TEST_FILES "" CACHE INTERNAL "")
include(CMakeParseArguments)

# Function to add google tests to src lists
function(add_google_tests test_files)

    cmake_parse_arguments(
        "${prefix}"
        ""
        ""
        "TEST_FILES"
    )

    # message("Provided sources are:")
    # foreach(src ${TEST_FILES})
    #     message("- ${src}")
    # endforeach(src)

    set(ALL_TEST_FILES 
        ${ALL_TEST_FILES}
        ${TEST_FILES}
        CACHE INTERNAL ""
    )
endfunction()

function(rollup_google_tests)

    set(ALL_TEST_FILES
        ${ALL_TEST_FILES}
        ${XCALIBRATE_SRC}
        ${VIZ_NODE_SRC}
        test/main.cpp)

    add_executable(ALL_GOOGLE_TESTS ${ALL_TEST_FILES})
    target_link_libraries(ALL_GOOGLE_TESTS gtest)

endfunction()

