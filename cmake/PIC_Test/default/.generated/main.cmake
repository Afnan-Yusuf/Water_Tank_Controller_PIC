include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")

set(PIC_Test_default_library_list )

# Handle files with suffix (s|as|asm|AS|ASM|As|aS|Asm), for group default-XC8
if(PIC_Test_default_default_XC8_FILE_TYPE_assemble)
add_library(PIC_Test_default_default_XC8_assemble OBJECT ${PIC_Test_default_default_XC8_FILE_TYPE_assemble})
    PIC_Test_default_default_XC8_assemble_rule(PIC_Test_default_default_XC8_assemble)
    list(APPEND PIC_Test_default_library_list "$<TARGET_OBJECTS:PIC_Test_default_default_XC8_assemble>")
endif()

# Handle files with suffix S, for group default-XC8
if(PIC_Test_default_default_XC8_FILE_TYPE_assemblePreprocess)
add_library(PIC_Test_default_default_XC8_assemblePreprocess OBJECT ${PIC_Test_default_default_XC8_FILE_TYPE_assemblePreprocess})
    PIC_Test_default_default_XC8_assemblePreprocess_rule(PIC_Test_default_default_XC8_assemblePreprocess)
    list(APPEND PIC_Test_default_library_list "$<TARGET_OBJECTS:PIC_Test_default_default_XC8_assemblePreprocess>")
endif()

# Handle files with suffix [cC], for group default-XC8
if(PIC_Test_default_default_XC8_FILE_TYPE_compile)
add_library(PIC_Test_default_default_XC8_compile OBJECT ${PIC_Test_default_default_XC8_FILE_TYPE_compile})
    PIC_Test_default_default_XC8_compile_rule(PIC_Test_default_default_XC8_compile)
    list(APPEND PIC_Test_default_library_list "$<TARGET_OBJECTS:PIC_Test_default_default_XC8_compile>")
endif()

add_executable(${PIC_Test_default_image_name} ${PIC_Test_default_library_list})

target_link_libraries(${PIC_Test_default_image_name} PRIVATE ${PIC_Test_default_default_XC8_FILE_TYPE_link})

# Add the link options from the rule file.
PIC_Test_default_link_rule(${PIC_Test_default_image_name})


# Post build target to copy built file to the output directory.
add_custom_command(TARGET ${PIC_Test_default_image_name} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E make_directory ${PIC_Test_default_output_dir}
                    COMMAND ${CMAKE_COMMAND} -E copy ${PIC_Test_default_image_name} ${PIC_Test_default_output_dir}/${PIC_Test_default_original_image_name}
                    BYPRODUCTS ${PIC_Test_default_output_dir}/${PIC_Test_default_original_image_name})
