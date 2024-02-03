
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_RV8803 rv8803.c)