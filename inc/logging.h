//
// Created by Kurosu Chan on 2024/1/14.
//

#ifndef LOGGING_H
#define LOGGING_H
#include <concepts>
#include <cinttypes>
#include <pico/time.h>
#include <sys/unistd.h>

/**
 *
 * Why do I need macro here?
 *
 * The initial implementation was done with C++ template and variadic parameter.
 * However `vprintf` would eats up too much memory. I could only use `printf` (which seems weird since `printf`
 * should have called `vprintf` but due to some compiler magic the `printf` eats 466 bytes and the `vprintf`
 * would eats several KBs)
 *
 * The unavailability of `vprintf` means variadic parameter could not be used. One has to pass the arguments directly
 * to `printf`. Ugly indeed...
 *
 */
namespace logging {
// somehow the Level is ordered
// the lower the level, the more verbose
enum class Level {
	Trace = 0,
	Debug,
	Info,
	Warn,
	Error,
	Off,
};

/**
 * @brief A concept that checks if a given callable object F is convertible to a std::function of type U.
 *
 * This concept is useful for checking if a given callable object (like a function, lambda, or functor)
 * can be stored in a std::function of a specific type. This can be used to enforce that template parameters
 * in a function or class template are callable objects that match a specific function signature.
 *
 * @tparam F The callable object to check.
 * @tparam U The function type to check against.
 */
template <auto F, typename U>
concept is_such_function_v = std::is_convertible_v<decltype(F), std::function<U>>;

constexpr const char *level_to_string(const Level level) {
	switch (level) {
	case Level::Trace:
		return "T";
	case Level::Debug:
		return "D";
	case Level::Info:
		return "I";
	case Level::Warn:
		return "W";
	case Level::Error:
		return "E";
	case Level::Off:
		return "";
	}
}

// https://askubuntu.com/questions/1405822/printf-statement-with-background-and-foreground-colours
constexpr const char *level_to_color(const Level level) {
	switch (level) {
	case Level::Trace:
		return "\033[37m";
	case Level::Debug:
		return "\033[34m";
	case Level::Info:
		return "\033[32m";
	case Level::Warn:
		return "\033[33m";
	case Level::Error:
		return "\033[31m";
	case Level::Off:
		return "";
	}
}


constexpr auto COLOR_RESET           = "\033[0m";
constexpr size_t LOGGING_USE_COLOR   = 1 << 0;
constexpr size_t LOGGING_NO_FILENAME = 1 << 1;

template <Level L>
void print_prefix(const char *tag, const char *file, const int line,
				  uint32_t timestamp,
				  int (*printer)(const char *...),
				  const size_t flags) {
	const auto is_use_color   = (flags & LOGGING_USE_COLOR) != 0;
	const auto is_no_filename = (flags & LOGGING_NO_FILENAME) != 0 or file == nullptr;
	if (tag == nullptr) {
		tag = "";
	}
	if (is_use_color) {
		if (is_no_filename) {
			printer("%" PRIu32 "\t%s[%s]%s%s\t", timestamp, level_to_color(L), level_to_string(L), tag, COLOR_RESET);
		} else {
			printer("%" PRIu32 "\t%s%s:%d\t[%s]%s%s\t", timestamp, level_to_color(L), file, line, level_to_string(L), tag, COLOR_RESET);
		}
	} else {
		if (is_no_filename) {
			printer("%" PRIu32 "\t[%s]%s\t", timestamp, level_to_string(L), tag);
		} else {
			printer("%" PRIu32 "\t%s:%d\t[%s]%s\t", timestamp, file, line, level_to_string(L), tag);
		}
	}
}

constexpr auto global_flags         = LOGGING_USE_COLOR;
constexpr auto global_logging_level = Level::Trace;
constexpr auto global_printer       = printf;
constexpr auto global_clock         = [] {
    constexpr auto MS_PER_US = 1'000;
    return static_cast<uint32_t>(time_us_64() / MS_PER_US);
};

#define LOG_GLOBAL(level, tag, file, line, fmt, ...)                                                                                \
	do {                                                                                                                            \
		if (level >= logging::global_logging_level) {                                                                               \
			logging::print_prefix<level>(tag, file, line, logging::global_clock(), logging::global_printer, logging::global_flags); \
			logging::global_printer(fmt, ##__VA_ARGS__);                                                                            \
			logging::global_printer("\n");                                                                                          \
		}                                                                                                                           \
	} while (0)

#define LOGI(tag, fmt, ...) LOG_GLOBAL(logging::Level::Info, tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGD(tag, fmt, ...) LOG_GLOBAL(logging::Level::Debug, tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGE(tag, fmt, ...) LOG_GLOBAL(logging::Level::Error, tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGW(tag, fmt, ...) LOG_GLOBAL(logging::Level::Warn, tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGP(tag, fmt, ...)            \
	do {                               \
		LOGE(tag, fmt, ##__VA_ARGS__); \
		_exit(1);                      \
	} while (0)
} // namespace logging
#endif // LOGGING_H
