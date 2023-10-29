set(SOURCES main.cpp)
add_executable(main ${SOURCES})
target_link_libraries(main soem alchemy copperplate cobalt)

set(OS "linux")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror ")
  set(OS_LIBS cobalt modechk pthread rt)
  execute_process(COMMAND xeno-config --posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  set(CMAKE_C_COMPILER gcc)


install(TARGETS main DESTINATION bin)

include_directories(
	
	/usr/xenomai/include
	/usr/xenomai/include/cobalt
	/usr/xenomai/include/alchemy
	)
	link_directories(/usr/xenomai/lib
	
			)
