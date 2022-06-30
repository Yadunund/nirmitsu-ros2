file(GLOB_RECURSE CPPS  ./*.cpp )

add_executable(connection_colors ${CPPS})

target_link_libraries(connection_colors nodes)

install(
  TARGETS connection_colors
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)