cmake_minimum_required(VERSION 3.10)
project(ModernRoboticsFinal)

set(CMAKE_CXX_STANDARD 17)

# חיפוש חבילת Eigen3
find_package(Eigen3 REQUIRED)

# הוספת תיקיית ה-include המקומית שלך
include_directories(include)

# יצירת קובץ ההרצה
add_executable(${PROJECT_NAME} src/main.cpp)

# קישור ל-Eigen
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)