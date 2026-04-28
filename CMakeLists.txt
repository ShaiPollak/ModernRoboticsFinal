cmake_minimum_required(VERSION 3.10)
project(ModernRoboticsFinal)

# תקן C++17 הכרחי עבור std::clamp ו-noexcept
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# חיפוש חבילת Eigen3
find_package(Eigen3 REQUIRED)

# הגדרת קבצי המקור (כדי שלא תצטרך לכתוב הכל בשורה אחת ארוכה)
set(SOURCES
    src/main.cpp
    src/robot_logger.cpp
    src/trajectory_gen.cpp
    src/robot_runner.cpp
    src/robot_kinematics.cpp
    src/robot_controller.cpp
)

# יצירת קובץ ההרצה
add_executable(${PROJECT_NAME} ${SOURCES})

# הגדרת תיקיות ה-Include
# ה-PRIVATE אומר שרק הפרויקט הזה משתמש בתיקיות האלו
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include 
    ${EIGEN3_INCLUDE_DIR}
)

# קישור לספריית Eigen
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)