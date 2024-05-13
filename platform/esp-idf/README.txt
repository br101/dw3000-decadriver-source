Since ESP-IDF always sets the components name to the name of the subfolder,
this is a workaround to get a component name of "decadriver". This name 
can be used in dependencies of other components.

Add the following to your main ESP-IDF CMakeLists.txt:

set(EXTRA_COMPONENT_DIRS components/dw3000-decadriver-source/platform/esp-idf)
