#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>

#define KERNEL_FILE_NAME "/dev/accel"

int main() {
    int fd = open(KERNEL_FILE_NAME, O_RDWR);
    printf("[DEBUG] open /dev/accel success!\n");

    uint8_t *ctrl_addr = mmap(NULL, 64, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    printf("[DEBUG] get ctrl_addr 0x%lx\n", (uint64_t)ctrl_addr);

    return 0;
}
