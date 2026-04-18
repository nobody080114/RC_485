set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

find_program(ARM_NONE_EABI_GCC
	NAMES ${TOOLCHAIN_PREFIX}gcc
	HINTS
		"$ENV{STM32CubeCLT_GCC_PATH}/bin"
		"$ENV{GCC_ARM_NONE_EABI_PATH}/bin"
		"$ENV{GCC_ARM_NONE_EABI_PATH}"
		"$ENV{USERPROFILE}/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/13.3.1+st.9/bin"
)

if(NOT ARM_NONE_EABI_GCC)
	message(FATAL_ERROR
		"arm-none-eabi-gcc not found. Please add GNU Arm Embedded toolchain to PATH, or set STM32CubeCLT_GCC_PATH / GCC_ARM_NONE_EABI_PATH.")
endif()

get_filename_component(ARM_NONE_EABI_BIN_DIR "${ARM_NONE_EABI_GCC}" DIRECTORY)

find_program(ARM_NONE_EABI_GXX
	NAMES ${TOOLCHAIN_PREFIX}g++ ${TOOLCHAIN_PREFIX}g++.exe
	HINTS "${ARM_NONE_EABI_BIN_DIR}"
)
find_program(ARM_NONE_EABI_OBJCOPY
	NAMES ${TOOLCHAIN_PREFIX}objcopy ${TOOLCHAIN_PREFIX}objcopy.exe
	HINTS "${ARM_NONE_EABI_BIN_DIR}"
)
find_program(ARM_NONE_EABI_SIZE
	NAMES ${TOOLCHAIN_PREFIX}size ${TOOLCHAIN_PREFIX}size.exe
	HINTS "${ARM_NONE_EABI_BIN_DIR}"
)

if(NOT ARM_NONE_EABI_GXX OR NOT ARM_NONE_EABI_OBJCOPY OR NOT ARM_NONE_EABI_SIZE)
	message(FATAL_ERROR
		"Found arm-none-eabi-gcc but missing companion tools (g++, objcopy, size) in ${ARM_NONE_EABI_BIN_DIR}.")
endif()

set(CMAKE_C_COMPILER                ${ARM_NONE_EABI_GCC})
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${ARM_NONE_EABI_GXX})
set(CMAKE_LINKER                    ${ARM_NONE_EABI_GXX})
set(CMAKE_OBJCOPY                   ${ARM_NONE_EABI_OBJCOPY})
set(CMAKE_SIZE                      ${ARM_NONE_EABI_SIZE})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32H723XG_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
