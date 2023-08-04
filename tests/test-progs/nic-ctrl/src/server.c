#include "../../../../src/dev/xdr/kfd_ioctl.hh"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdint.h>
#include <string.h>

#define KERNEL_FILE_NAME "/dev/accel"

int main() {
    /*printf("This is an infinite loop ...\n");*/

    int fd = open(KERNEL_FILE_NAME, O_RDWR);
    printf("[DEBUG] open /dev/accel success!\n");

    uint8_t *ctrl_addr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    printf("[DEBUG] get ctrl_addr 0x%lx\n", (uint64_t)ctrl_addr);

    ioctl(fd, ACCELKFD_START_SVR, NULL);

    while (1) {
        sleep(1);
    }
    return 0;
}
