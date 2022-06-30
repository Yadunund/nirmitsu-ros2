file(GLOB_RECURSE CPPS  ./*.cpp )

add_executable(calculator ${CPPS})

target_link_libraries(calculator nodes)

install(
  TARGETS calculator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)