/* Isolated test process clock only. Never preload into robot/cloud processes.
 * WSL evidence showed CLOCK_REALTIME jumps while CLOCK_MONOTONIC was continuous.
 * Preserve the original controller freshness limits using one shared fixed offset.
 * Build: cc -shared -fPIC -O2 -Wall -Wextra -o /tmp/task6_fixture_clock.so
 *        test/task6_fixture_clock.c -ldl
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

int clock_gettime(clockid_t id, struct timespec *value) {
    static int (*original)(clockid_t, struct timespec *);
    if (!original) original = dlsym(RTLD_NEXT, "clock_gettime");
    const char *setting = getenv("TASK6_TEST_REALTIME_OFFSET_NS");
    if (!setting || (id != CLOCK_REALTIME && id != CLOCK_REALTIME_COARSE))
        return original(id, value);
    int result = original(CLOCK_MONOTONIC, value);
    if (!result) {
        int64_t ns = (int64_t)value->tv_sec * 1000000000LL + value->tv_nsec + strtoll(setting, NULL, 10);
        value->tv_sec = ns / 1000000000LL;
        value->tv_nsec = ns % 1000000000LL;
    }
    return result;
}

int gettimeofday(struct timeval *value, void *zone) {
    (void)zone;
    struct timespec now;
    int result = clock_gettime(CLOCK_REALTIME, &now);
    if (!result) {
        value->tv_sec = now.tv_sec;
        value->tv_usec = now.tv_nsec / 1000;
    }
    return result;
}

time_t time(time_t *value) {
    struct timespec now;
    if (clock_gettime(CLOCK_REALTIME, &now)) return (time_t)-1;
    if (value) *value = now.tv_sec;
    return now.tv_sec;
}