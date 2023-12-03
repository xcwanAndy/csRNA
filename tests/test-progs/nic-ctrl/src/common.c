#include "common.h"
#include <stddef.h>
#include <stdint.h>

uint32_t fill_read_mr(uint8_t *addr, uint32_t offset, char *payload, size_t payload_len) {

    // Write data to mr
    char *string = (char *)(addr + offset);
    memcpy(string, payload, payload_len);

    fprintf(stderr, "init_snd_wqe: string is %s, string vaddr is 0x%lx, start vaddr is 0x%lx\n",
            string, (uint64_t)string, (uint64_t)(addr + offset));

    offset += sizeof(*payload);
    return offset;
}


struct accelkfd_ioctl_mr_addr *set_mr_addr_args(uint8_t side, size_t len, int is_fill) {
    size_t mem_len = len;
    struct accelkfd_ioctl_mr_addr *args =
        (struct accelkfd_ioctl_mr_addr *)malloc(sizeof(struct accelkfd_ioctl_mr_addr));

    uint8_t *mr_addr = (uint8_t *)memalign(1<<12, mem_len);
    memset(mr_addr, 0, mem_len);
    printf("[DEBUG] allocated addr: %p\n", mr_addr);
    args->side = side;
    args->vaddr = (uint64_t)mr_addr;
    args->paddr = 0;
    args->size = mem_len;

    if (is_fill) {
        fill_read_mr(mr_addr, 0, DATA_8B, sizeof(DATA_8B));
        /*fill_read_mr(mr_addr, 0, DATA_64B, sizeof(DATA_64B));*/
    }

    return args;
}

