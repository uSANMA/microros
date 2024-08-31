idf_component_register(SRCS main.c uros.c sensors.c lidar.c http.c 
                        INCLUDE_DIRS "" "web"
                        EMBED_FILES  web/home.html web/ota.html web/restart.html web/uwaba.png web/mini.css web/styles.css web/menu.html
                        )

