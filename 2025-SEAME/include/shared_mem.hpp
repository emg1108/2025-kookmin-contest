#ifndef SHARED_MEM_HPP
#define SHARED_MEM_HPP

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <atomic>

#define SHM_NAME "/steering_throttle_shm"

struct SharedData {
    float steering;
    float throttle;
};

#endif
