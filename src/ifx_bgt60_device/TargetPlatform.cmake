set(platformdetect_c_code "
#if defined(__linux__) || defined(linux) || defined(__linux)
    #if defined(__x86_64) || defined(__x86_64__) || defined(__amd64) || defined(_M_X64)
        #error CMAKE_ARCH linux_x64
    #elif defined(__arm__)
        #error CMAKE_ARCH raspi
    #elif defined(__aarch64__)
        #error CMAKE_ARCH linux_aarch64
    #else
        #error CMAKE_ARCH unknown linux
    #endif
#elif defined(_WIN32) || defined(__MINGW64__) || defined(__CYGWIN__)
    #if defined(__i386) || defined(__i386__) || defined(_M_IX86)
        #error CMAKE_ARCH win32_x86
    #elif defined(__x86_64) || defined(__x86_64__) || defined(__amd64) || defined(_M_X64)
        #error CMAKE_ARCH win32_x64
    #else
        #error CMAKE_ARCH unknown windows
    #endif
#else
    #error CMAKE_ARCH unknown
#endif
")
function(detect_target_platform output_var)
    file(WRITE "${CMAKE_BINARY_DIR}/arch.c" "${platformdetect_c_code}")

    enable_language(C)

    # Detect the architecture in a rather creative way...
    # This compiles a small C program which is a series of ifdefs that selects a
    # particular #error preprocessor directive whose message string contains the
    # target architecture. The program will always fail to compile (both because
    # file is not a valid C program, and obviously because of the presence of the
    # #error preprocessor directives... but by exploiting the preprocessor in this
    # way, we can detect the correct target architecture even when cross-compiling,
    # since the program itself never needs to be run (only the compiler/preprocessor)
    try_run(
        run_result_unused
        compile_result_unused
        "${CMAKE_BINARY_DIR}"
        "${CMAKE_BINARY_DIR}/arch.c"
        COMPILE_OUTPUT_VARIABLE ARCH
    )

    # Parse the architecture name from the compiler output
    string(REGEX MATCH "CMAKE_ARCH ([a-zA-Z0-9_]+)" ARCH "${ARCH}")

    # Get rid of the value marker leaving just the architecture name
    string(REPLACE "CMAKE_ARCH " "" ARCH "${ARCH}")

    # If we are compiling with an unknown architecture this variable should
    # already be set to "unknown" but in the case that it's empty (i.e. due
    # to a typo in the code), then set it to unknown
    if (NOT ARCH)
        set(ARCH unknown)
    endif()

    set(${output_var} "${ARCH}" PARENT_SCOPE)
endfunction()
