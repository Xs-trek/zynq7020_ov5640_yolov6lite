#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/input.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_EVENT_NAME "gpio-keys"
#define DEFAULT_KEY_CODE 148
#define DEFAULT_CAM_BINARY "/usr/bin/cam_system"
#define DEFAULT_CAM_LOG "/tmp/cam_system.log"
#define MAX_INPUT_DEVICES 32
#define INPUT_NAME_LEN 128
#define BIT_WORD_BITS ((unsigned int)(sizeof(unsigned long) * CHAR_BIT))

struct watched_input {
    int fd;
    char path[32];
    char name[INPUT_NAME_LEN];
};

static volatile sig_atomic_t g_stop = 0;
static pid_t g_last_started_pid = -1;

static void on_signal(int signo)
{
    (void)signo;
    g_stop = 1;
}

static void log_msg(const char *fmt, ...)
{
    time_t now = time(NULL);
    struct tm tm_now;
    char stamp[32] = "unknown-time";

    if (now != (time_t)-1 && localtime_r(&now, &tm_now) != NULL) {
        strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &tm_now);
    }

    fprintf(stdout, "[%s] ", stamp);

    va_list ap;
    va_start(ap, fmt);
    vfprintf(stdout, fmt, ap);
    va_end(ap);

    fputc('\n', stdout);
    fflush(stdout);
}

static long env_long(const char *name, long fallback, long min_value, long max_value)
{
    const char *value = getenv(name);
    if (value == NULL || *value == '\0') {
        return fallback;
    }

    errno = 0;
    char *end = NULL;
    long parsed = strtol(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' ||
        parsed < min_value || parsed > max_value) {
        log_msg("invalid %s=%s, using %ld", name, value, fallback);
        return fallback;
    }

    return parsed;
}

static int bit_is_set(unsigned int bit, const unsigned long *bits, size_t words)
{
    size_t word = bit / BIT_WORD_BITS;
    unsigned int shift = bit % BIT_WORD_BITS;

    if (word >= words) {
        return 0;
    }

    return ((bits[word] >> shift) & 1UL) != 0;
}

static int input_has_event(int fd, unsigned int event_type)
{
    unsigned long bits[(EV_MAX / BIT_WORD_BITS) + 1];
    memset(bits, 0, sizeof(bits));

    if (ioctl(fd, EVIOCGBIT(0, sizeof(bits)), bits) < 0) {
        return 0;
    }

    return bit_is_set(event_type, bits, sizeof(bits) / sizeof(bits[0]));
}

static int input_has_key(int fd, unsigned int key_code)
{
    unsigned long bits[(KEY_MAX / BIT_WORD_BITS) + 1];
    memset(bits, 0, sizeof(bits));

    if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(bits)), bits) < 0) {
        return 0;
    }

    return bit_is_set(key_code, bits, sizeof(bits) / sizeof(bits[0]));
}

static int process_comm_equals(pid_t pid, const char *expected)
{
    char path[64];
    int n = snprintf(path, sizeof(path), "/proc/%ld/comm", (long)pid);
    if (n < 0 || (size_t)n >= sizeof(path)) {
        return 0;
    }

    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        return 0;
    }

    char comm[64];
    char *line = fgets(comm, sizeof(comm), fp);
    fclose(fp);

    if (line == NULL) {
        return 0;
    }

    comm[strcspn(comm, "\r\n")] = '\0';
    return strcmp(comm, expected) == 0;
}

static int cam_system_running(void)
{
    DIR *proc = opendir("/proc");
    if (proc == NULL) {
        log_msg("failed to open /proc: %s", strerror(errno));
        return 0;
    }

    struct dirent *entry;
    while ((entry = readdir(proc)) != NULL) {
        const char *name = entry->d_name;
        if (!isdigit((unsigned char)name[0])) {
            continue;
        }

        char *end = NULL;
        errno = 0;
        long pid_long = strtol(name, &end, 10);
        if (errno != 0 || end == name || *end != '\0' ||
            pid_long <= 0 || pid_long > INT_MAX) {
            continue;
        }

        if (process_comm_equals((pid_t)pid_long, "cam_system")) {
            closedir(proc);
            return 1;
        }
    }

    closedir(proc);
    return 0;
}

static int last_started_process_running(void)
{
    if (g_last_started_pid <= 0) {
        return 0;
    }

    if (kill(g_last_started_pid, 0) == 0) {
        return 1;
    }

    if (errno == ESRCH) {
        g_last_started_pid = -1;
    }

    return 0;
}

static int redirect_child_stdio(const char *log_path)
{
    int null_fd = open("/dev/null", O_RDONLY | O_CLOEXEC);
    if (null_fd < 0) {
        return -1;
    }

    if (dup2(null_fd, STDIN_FILENO) < 0) {
        close(null_fd);
        return -1;
    }
    close(null_fd);

    int log_fd = open(log_path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (log_fd < 0) {
        return -1;
    }

    if (dup2(log_fd, STDOUT_FILENO) < 0 ||
        dup2(log_fd, STDERR_FILENO) < 0) {
        close(log_fd);
        return -1;
    }

    close(log_fd);
    return 0;
}

static int launch_cam_system(const char *binary, const char *log_path)
{
    if (cam_system_running() || last_started_process_running()) {
        log_msg("cam_system already running; key press ignored");
        return 0;
    }

    if (access(binary, X_OK) != 0) {
        log_msg("cam_system binary is not executable: %s: %s",
                binary, strerror(errno));
        return -1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        log_msg("fork failed: %s", strerror(errno));
        return -1;
    }

    if (pid == 0) {
        if (setsid() < 0) {
            _exit(126);
        }

        if (redirect_child_stdio(log_path) < 0) {
            _exit(126);
        }

        execl(binary, binary, (char *)NULL);
        _exit(127);
    }

    g_last_started_pid = pid;
    log_msg("started cam_system pid=%ld", (long)pid);
    return 0;
}

static int scan_inputs(struct watched_input *inputs,
                       size_t capacity,
                       const char *name_filter,
                       unsigned int key_code)
{
    size_t count = 0;

    for (int i = 0; i < MAX_INPUT_DEVICES && count < capacity; ++i) {
        char path[32];
        int n = snprintf(path, sizeof(path), "/dev/input/event%d", i);
        if (n < 0 || (size_t)n >= sizeof(path)) {
            continue;
        }

        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            continue;
        }

        char name[INPUT_NAME_LEN] = "";
        if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) {
            close(fd);
            continue;
        }

        if (name_filter[0] != '\0' && strstr(name, name_filter) == NULL) {
            close(fd);
            continue;
        }

        int has_event = input_has_event(fd, EV_KEY);
        int has_key = input_has_key(fd, key_code);
        if (!has_event || !has_key) {
            log_msg("input candidate %s name='%s' lacks event/key: EV_KEY=%d key_%u=%d",
                    path, name, has_event, key_code, has_key);
            close(fd);
            continue;
        }

        inputs[count].fd = fd;
        snprintf(inputs[count].path, sizeof(inputs[count].path), "%s", path);
        snprintf(inputs[count].name, sizeof(inputs[count].name), "%s", name);
        ++count;
    }

    return (int)count;
}

static void close_inputs(struct watched_input *inputs, int count)
{
    for (int i = 0; i < count; ++i) {
        if (inputs[i].fd >= 0) {
            close(inputs[i].fd);
            inputs[i].fd = -1;
        }
    }
}

static void handle_input_event(const struct input_event *event,
                               unsigned int key_code,
                               const char *binary,
                               const char *log_path)
{
    if (event->type != EV_KEY || event->code != key_code) {
        return;
    }

    if (event->value == 1) {
        log_msg("key code %u pressed", key_code);
        (void)launch_cam_system(binary, log_path);
    }
}

static void watch_inputs(const char *name_filter,
                         unsigned int key_code,
                         const char *binary,
                         const char *log_path)
{
    while (!g_stop) {
        struct watched_input inputs[MAX_INPUT_DEVICES];
        for (int i = 0; i < MAX_INPUT_DEVICES; ++i) {
            inputs[i].fd = -1;
        }

        int count = scan_inputs(inputs, MAX_INPUT_DEVICES, name_filter, key_code);
        if (count <= 0) {
            log_msg("no input device found for name='%s' key=%u; rescanning",
                    name_filter, key_code);
            sleep(2);
            continue;
        }

        for (int i = 0; i < count; ++i) {
            log_msg("watching %s name='%s' key=%u",
                    inputs[i].path, inputs[i].name, key_code);
        }

        struct pollfd fds[MAX_INPUT_DEVICES];
        for (int i = 0; i < count; ++i) {
            fds[i].fd = inputs[i].fd;
            fds[i].events = POLLIN;
            fds[i].revents = 0;
        }

        while (!g_stop) {
            int rc = poll(fds, (nfds_t)count, 2000);
            if (rc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                log_msg("poll failed: %s", strerror(errno));
                break;
            }

            if (rc == 0) {
                continue;
            }

            int need_rescan = 0;
            for (int i = 0; i < count; ++i) {
                if ((fds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
                    need_rescan = 1;
                    continue;
                }

                if ((fds[i].revents & POLLIN) == 0) {
                    continue;
                }

                for (;;) {
                    struct input_event event;
                    ssize_t n = read(fds[i].fd, &event, sizeof(event));
                    if (n < 0) {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) {
                            break;
                        }
                        log_msg("read %s failed: %s",
                                inputs[i].path, strerror(errno));
                        need_rescan = 1;
                        break;
                    }

                    if (n != (ssize_t)sizeof(event)) {
                        log_msg("short read from %s", inputs[i].path);
                        need_rescan = 1;
                        break;
                    }

                    handle_input_event(&event, key_code, binary, log_path);
                }
            }

            if (need_rescan) {
                break;
            }
        }

        close_inputs(inputs, count);
    }
}

int main(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);

    struct sigaction child_sa;
    memset(&child_sa, 0, sizeof(child_sa));
    child_sa.sa_handler = SIG_IGN;
    child_sa.sa_flags = SA_NOCLDWAIT;
    sigemptyset(&child_sa.sa_mask);
    sigaction(SIGCHLD, &child_sa, NULL);

    const char *name_filter = getenv("CAM_LAUNCHER_EVENT_NAME");
    if (name_filter == NULL) {
        name_filter = DEFAULT_EVENT_NAME;
    }

    long key_code_long = env_long("CAM_LAUNCHER_KEY_CODE",
                                  DEFAULT_KEY_CODE, 0, KEY_MAX);
    unsigned int key_code = (unsigned int)key_code_long;

    const char *binary = getenv("CAM_LAUNCHER_CAM_BINARY");
    if (binary == NULL || *binary == '\0') {
        binary = DEFAULT_CAM_BINARY;
    }

    const char *log_path = getenv("CAM_LAUNCHER_CAM_LOG");
    if (log_path == NULL || *log_path == '\0') {
        log_path = DEFAULT_CAM_LOG;
    }

    log_msg("cam-launcher starting: event_name='%s' key=%u binary=%s",
            name_filter, key_code, binary);
    watch_inputs(name_filter, key_code, binary, log_path);
    log_msg("cam-launcher exiting");

    return 0;
}
