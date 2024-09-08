cmake_minimum_required(VERSION 3.28)
project(strela)

# Nastavení standardu C++
set(CMAKE_CXX_STANDARD 20)

# Přidání EnTT jako subdirectory
add_subdirectory(extern/entt)

# Přidání spustitelného souboru
add_executable(strela main.cpp)

# Propojení EnTT knihovny s executable
target_link_libraries(strela PRIVATE EnTT)
