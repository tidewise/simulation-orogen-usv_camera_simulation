#ifndef usv_camera_simulation_TYPES_HPP
#define usv_camera_simulation_TYPES_HPP

#include <string>

namespace usv_camera_simulation {
    struct ModelDefinition {
        /** Minimum size for this SDF model */
        float min_size = 0;
        /** Maximum size for this SDF model */
        float max_size = 500;
        /** Path to the 3D model */
        std::string model;
    };
}

#endif
