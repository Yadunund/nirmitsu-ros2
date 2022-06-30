file(GLOB_RECURSE CPPS  ./*.cpp )

add_executable(example2 ${CPPS})

target_link_libraries(example2 nodes)

install(
  TARGETS example2
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)