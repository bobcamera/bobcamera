#pragma once

#include <string>

namespace boblib::utils::console_colors
{
    // Reset
    constexpr char RESET[] = "\033[0m";

    // Regular text colors
    constexpr char BLACK[] = "\033[30m";
    constexpr char RED[] = "\033[31m";
    constexpr char GREEN[] = "\033[32m";
    constexpr char YELLOW[] = "\033[33m";
    constexpr char BLUE[] = "\033[34m";
    constexpr char MAGENTA[] = "\033[35m";
    constexpr char CYAN[] = "\033[36m";
    constexpr char WHITE[] = "\033[37m";

    // Bright text colors
    constexpr char BRIGHT_BLACK[] = "\033[90m";
    constexpr char BRIGHT_RED[] = "\033[91m";
    constexpr char BRIGHT_GREEN[] = "\033[92m";
    constexpr char BRIGHT_YELLOW[] = "\033[93m";
    constexpr char BRIGHT_BLUE[] = "\033[94m";
    constexpr char BRIGHT_MAGENTA[] = "\033[95m";
    constexpr char BRIGHT_CYAN[] = "\033[96m";
    constexpr char BRIGHT_WHITE[] = "\033[97m";

    // Background colors
    constexpr char BG_BLACK[] = "\033[40m";
    constexpr char BG_RED[] = "\033[41m";
    constexpr char BG_GREEN[] = "\033[42m";
    constexpr char BG_YELLOW[] = "\033[43m";
    constexpr char BG_BLUE[] = "\033[44m";
    constexpr char BG_MAGENTA[] = "\033[45m";
    constexpr char BG_CYAN[] = "\033[46m";
    constexpr char BG_WHITE[] = "\033[47m";

    // Bright background colors
    constexpr char BG_BRIGHT_BLACK[] = "\033[100m";
    constexpr char BG_BRIGHT_RED[] = "\033[101m";
    constexpr char BG_BRIGHT_GREEN[] = "\033[102m";
    constexpr char BG_BRIGHT_YELLOW[] = "\033[103m";
    constexpr char BG_BRIGHT_BLUE[] = "\033[104m";
    constexpr char BG_BRIGHT_MAGENTA[] = "\033[105m";
    constexpr char BG_BRIGHT_CYAN[] = "\033[106m";
    constexpr char BG_BRIGHT_WHITE[] = "\033[107m";

    // Text styles
    constexpr char BOLD[] = "\033[1m";
    constexpr char DIM[] = "\033[2m";
    constexpr char ITALIC[] = "\033[3m";
    constexpr char UNDERLINE[] = "\033[4m";
    constexpr char BLINK[] = "\033[5m";
    constexpr char INVERT[] = "\033[7m";
    constexpr char HIDDEN[] = "\033[8m";
    constexpr char STRIKETHROUGH[] = "\033[9m";
    constexpr char UNDERLINE_DOUBLE[] = "\033[21m";
}