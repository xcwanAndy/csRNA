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
    size_t mem_len = 1024;
    struct accelkfd_ioctl_mr_addr *args =
        (struct accelkfd_ioctl_mr_addr *) malloc(sizeof(struct accelkfd_ioctl_mr_addr));

    int fd = open(KERNEL_FILE_NAME, O_RDWR);
    printf("[DEBUG] open /dev/accel success!\n");

    uint8_t *ctrl_addr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    printf("[DEBUG] get ctrl_addr 0x%lx\n", (uint64_t)ctrl_addr);

    uint8_t *mr_addr = (uint8_t *)malloc(mem_len);
    memset(mr_addr, 0, mem_len);
    printf("[DEBUG] allocated addr: %p\n", mr_addr);
    args->side = CLIENT_SIDE;
    args->vaddr = (uint64_t)mr_addr;
    args->paddr = 0;
    args->size = mem_len;

    /*memcpy(ctrl_addr, args, sizeof(struct kfd_ioctl_mr_addr));*/

    ioctl(fd, ACCELKFD_SEND_MR_ADDR, (void *)args);

    ioctl(fd, ACCELKFD_START_CLT, NULL);

    while (1) {
        sleep(2);
        if (*mr_addr != 0) {
            printf("The string in mr_addr: %s", (char*)mr_addr);
        }
    }

    return 0;
}
