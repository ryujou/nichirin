cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32G431xx
    LV_CONF_INCLUDE_SIMPLE
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_Device/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_Device/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32G4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_Device/App/usb_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_Device/App/usbd_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_Device/App/usbd_cdc_if.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_Device/Target/usbd_conf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/flash_cfg.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/st7735.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/app/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/app/mode_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/led/led_ctrl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/led/led_engine.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/led/led_effects.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/storage/config_store.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/ui/ui.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/ui/ui_led_menu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/ui/lcd_port.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/ui/keypad_port.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stream/video_proto.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stream/usb_stream_rx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stream/lcd_streamer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stream/spectrum_led.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32g4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32g4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32g431xx.s
)

file(GLOB LVGL_Core_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/core/*.c
)
file(GLOB LVGL_Root_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/*.c
)
file(GLOB LVGL_Display_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/display/*.c
)
file(GLOB LVGL_Draw_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/draw/*.c
)
file(GLOB LVGL_Draw_Sw_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/draw/sw/*.c
)
file(GLOB LVGL_Draw_Sw_Blend_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/draw/sw/blend/*.c
)
file(GLOB LVGL_Indev_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/indev/*.c
)
file(GLOB LVGL_Layouts_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/layouts/*.c
)
file(GLOB LVGL_Misc_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/misc/*.c
)
file(GLOB LVGL_Misc_Cache_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/misc/cache/*.c
)
file(GLOB LVGL_Libs_Bin_Decoder_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/libs/bin_decoder/*.c
)
file(GLOB LVGL_Stdlib_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/stdlib/*.c
)
file(GLOB LVGL_Stdlib_Builtin_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/stdlib/builtin/*.c
)
file(GLOB LVGL_Tick_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/tick/*.c
)
file(GLOB LVGL_Widgets_Button_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/widgets/button/*.c
)
file(GLOB LVGL_Widgets_ButtonMatrix_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/widgets/buttonmatrix/*.c
)
file(GLOB LVGL_Widgets_Label_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/widgets/label/*.c
)
file(GLOB LVGL_Themes_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/themes/*.c
)

set(LVGL_OSAL_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/osal/lv_os_none.c
)

set(LVGL_Font_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/font/lv_font.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/font/lv_font_fmt_txt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/lvgl/src/font/lv_font_montserrat_14.c
)

set(LVGL_Src
    ${LVGL_Core_Src}
    ${LVGL_Root_Src}
    ${LVGL_Display_Src}
    ${LVGL_Draw_Src}
    ${LVGL_Draw_Sw_Src}
    ${LVGL_Draw_Sw_Blend_Src}
    ${LVGL_Indev_Src}
    ${LVGL_Layouts_Src}
    ${LVGL_Misc_Src}
    ${LVGL_Misc_Cache_Src}
    ${LVGL_Libs_Bin_Decoder_Src}
    ${LVGL_Stdlib_Src}
    ${LVGL_Stdlib_Builtin_Src}
    ${LVGL_Tick_Src}
    ${LVGL_OSAL_Src}
    ${LVGL_Font_Src}
    ${LVGL_Widgets_Button_Src}
    ${LVGL_Widgets_ButtonMatrix_Src}
    ${LVGL_Widgets_Label_Src}
    ${LVGL_Themes_Src}
)
list(APPEND MX_Application_Src ${LVGL_Src})

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32g4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
)

# Drivers Midllewares


set(USB_Device_Library_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
    USB_Device_Library
	
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create USB_Device_Library static library
add_library(USB_Device_Library OBJECT)
target_sources(USB_Device_Library PRIVATE ${USB_Device_Library_Src})
target_link_libraries(USB_Device_Library PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
