file(GLOB DRIVER_SRCS "./Drivers/src/*.cpp")
file(GLOB MIDDLEWARE_SRCS "./Middlewares/src/*.cpp")
file(GLOB MODULE_SRCS "./Modules/src/*.cpp")
file(GLOB APPLICATION_SRCS "./Applications/src/*.cpp")
# file(GLOB PICTURE_SRCS "./picture/*.c")
idf_component_register(SRCS "main.c"
                            ${DRIVER_SRCS}
                            ${MIDDLEWARE_SRCS}
                            ${MODULE_SRCS}
                            ${APPLICATION_SRCS}
                    INCLUDE_DIRS    "./Drivers/include"
                                    "./Middlewares/include"
                                    "./Modules/include"
                                    "./Applications/include"
                                    )
