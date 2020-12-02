set(COMPONENT_SRCS "espcam7670.c" "tcpserver.c")

set(COMPONENT_REQUIRES
    esp32-camera-master
    nvs_flash
    )

set(COMPONENT_EMBED_FILES
        "www/index_ov7670.html.gz"

register_component()
