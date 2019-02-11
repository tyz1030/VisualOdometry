find_package( G2O REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Cholmod )

aux_source_directory(. DIR_SRCS)

set(EXECUTABLE_OUTPUT_PATH "../")
add_executable(visualOdometry ${DIR_SRCS})

target_link_libraries( visualOdometry 
    ${OpenCV_LIBS}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
    ${CHOLMOD_LIBRARIES}
    )