#pragma once

#include <string>

namespace boblib::base
{
    class Utils final
    {
    public:
        static bool has_cuda();

        static bool has_opencl();

        static std::string get_ocl_info();

        static void reset_cuda();

        static std::string get_cpu_name();

        static unsigned long get_memory_total();

        static size_t get_available_threads();
    };
}
