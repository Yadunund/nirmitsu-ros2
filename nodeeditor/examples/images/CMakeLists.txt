file(GLOB_RECURSE CPPS  ./*.cpp )

add_executable(images ${CPPS})

target_link_libraries(images nodes)

install(
  TARGETS images
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)