#include <cstdlib>
#include <dlfcn.h>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char ** argv) {
    std::vector<std::string> candidates;
    if (argc > 1 && argv[1] != nullptr && argv[1][0] != '\0') {
        candidates.emplace_back(argv[1]);
    }

    if (const char * env_lib = std::getenv("CUVSLAM_LIBRARY")) {
        if (env_lib[0] != '\0') {
            candidates.emplace_back(env_lib);
        }
    }

    if (const char * env_dst = std::getenv("CUVSLAM_DST_DIR")) {
        if (env_dst[0] != '\0') {
            candidates.emplace_back(std::string(env_dst) + "/bin/libcuvslam.so");
        }
    }

    candidates.emplace_back("/workspace/src/cuVSLAM/build/bin/libcuvslam.so");
    candidates.emplace_back("/docker_cache/cuvslam/x86_64/current/install/lib/libcuvslam.so");
    candidates.emplace_back("/workspace/docker_cache/cuvslam/x86_64/current/install/lib/libcuvslam.so");

    for (const auto & lib_path : candidates) {
        if (!std::filesystem::exists(lib_path)) {
            continue;
        }

        void * handle = dlopen(lib_path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (handle != nullptr) {
            std::cout << "cuVSLAM library loaded successfully from " << lib_path << std::endl;
            dlclose(handle);
            return 0;
        }

        std::cerr << "Failed to load cuVSLAM library from " << lib_path
                  << ": " << dlerror() << std::endl;
    }

    std::cerr << "Failed to load cuVSLAM library from any known location." << std::endl;
    return 1;
}
