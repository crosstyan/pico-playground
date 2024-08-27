//
// Created by Kurosu Chan on 2024/1/14.
//

#ifndef LOGGING_H
#define LOGGING_H
#include <concepts>
#include <cinttypes>
#include <pico/time.h>
#include <sys/unistd.h>

namespace logging {
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

/**
 * @brief Print the prefix of a log message
 * @tparam L log level
 * @param tag tag for the log message
 * @param file file name of the log message
 * @param line line number of the log message
 * @param timestamp timestamp of the log message
 * @param printer a function pointer to print the log message, printf like
 * @param flags see `LOGGING_*`
 */
template <Level L>
void print_prefix(const char *tag, const char *file, const int line,
				  const uint32_t timestamp,
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
			logging::global_printer(fmt "\r\n", ##__VA_ARGS__);                                                                     \
		}                                                                                                                           \
	} while (0)

#define LOGI(tag, fmt, ...) LOG_GLOBAL(logging::Level::Info, tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGD(tag, fmt, ...) LOG_GLOBAL(logging::Level::Debug, tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGE(tag, fmt, ...) LOG_GLOBAL(logging::Level::Error, tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
#define LOGW(tag, fmt, ...) LOG_GLOBAL(logging::Level::Warn, tag, __FILE_NAME__, __LINE__, fmt, ##__VA_ARGS__)
/**
 * @brief Log an error message and exit the program (panic)
 */
#define LOGP(tag, fmt, ...)            \
	do {                               \
		LOGE(tag, fmt, ##__VA_ARGS__); \
		_exit(1);                      \
	} while (0)
} // namespace logging
#endif // LOGGING_H
