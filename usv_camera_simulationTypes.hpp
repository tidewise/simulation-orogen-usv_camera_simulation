#ifndef usv_camera_simulation_TYPES_HPP
#define usv_camera_simulation_TYPES_HPP

#include <string>

namespace usv_camera_simulation {
    struct ModelDefinition {
        /** Minimum size for which this SDF model may be selected */
        float min_size = 0;
        /** Maximum size for which this SDF model may be selected */
        float max_size = 500;
        /** SDF description of the model */
        std::string sdf;
        /** SDF version used */
        std::string sdf_version = "1.6";
    };
}

#endif
