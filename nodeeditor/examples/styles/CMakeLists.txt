file(GLOB_RECURSE CPPS  ./*.cpp )

add_executable(styles ${CPPS})

target_link_libraries(styles nodes)

install(
  TARGETS styles
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)