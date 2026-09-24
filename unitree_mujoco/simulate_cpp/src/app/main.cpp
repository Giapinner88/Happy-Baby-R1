#include "app/PolicyApplication.hpp"

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
    try {
        return RunPolicyApplication(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "[Fatal] " << error.what() << '\n';
        return 1;
    }
}
