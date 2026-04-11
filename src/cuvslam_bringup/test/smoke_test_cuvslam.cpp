#include <iostream>
#include <dlfcn.h>

int main() {
    const char* lib_path = "/workspace/docker_cache/cuvslam/x86_64/current/install/lib/libcuvslam.so";
    void* handle = dlopen(lib_path, RTLD_LAZY);
    if (!handle) {
        std::cerr << "Failed to load cuVSLAM library: " << dlerror() << std::endl;
        return 1;
    }
    std::cout << "cuVSLAM library loaded successfully!" << std::endl;
    dlclose(handle);
    return 0;
}
