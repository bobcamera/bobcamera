#pragma once

namespace boblib::base
{
    class Utils
    {
    public:
        static bool HasCuda();

        static void ResetCuda();
    };
}
