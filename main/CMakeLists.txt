file(GLOB DRIVER_SRCS "./Drivers/src/*.cpp")
# file(GLOB MIDDLEWARE_SRCS "./Middlewares/src/*.cpp")
file(GLOB MODULE_SRCS "./Modules/src/*.cpp")
file(GLOB APPLICATION_SRCS "./Applications/src/*.cpp")
file(GLOB UTILITY_SRCS "./Utility/src/*.cpp")
file(GLOB PLATFORM_SRCS "./Platform/src/*.cpp")
# file(GLOB PICTURE_SRCS "./picture/*.c")
idf_component_register(SRCS "main.c"
                            ${DRIVER_SRCS}
                        #     ${MIDDLEWARE_SRCS}
                            ${MODULE_SRCS}
                            ${APPLICATION_SRCS}
                            ${UTILITY_SRCS}
                            ${PLATFORM_SRCS}
                    INCLUDE_DIRS    "./Drivers/include"
                                #     "./Middlewares/include"
                                    "./Modules/include"
                                    "./Applications/include"
                                    "./Utility/include"
                                    "./Platform/include"
                                    )
