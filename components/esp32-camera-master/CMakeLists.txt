set(COMPONENT_SRCS
  driver/camera.c
  driver/sccb.c
  driver/sensor.c
  driver/twi.c
  driver/xclk.c
  sensors/ov7670.c
  conversions/to_jpg.cpp
  )

set(COMPONENT_ADD_INCLUDEDIRS
  driver/include
  conversions/include
  )

set(COMPONENT_PRIV_INCLUDEDIRS
  driver/private_include
  sensors/private_include
  conversions/private_include
  )

set(COMPONENT_REQUIRES driver)
set(COMPONENT_PRIV_REQUIRES freertos nvs_flash)

register_component()
