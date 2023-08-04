#ifndef __ACCEL_DEFS_HH__
#define __ACCEL_DEFS_HH__

#include <cstdint>
#include <deque>
#include <iterator>
#include <queue>
#include <string>
#include <list>
#include <unordered_map>

#include "base/addr_range.hh"
#include "dev/rdma/hangu_rnic_defs.hh"

typedef struct {
    bool isValid;
    AddrRange vaddr;
    AddrRange paddr;
}MemBlock;

class MemAllocator {
    private:
        Addr baseAddr;
        // Ordered by paddr
        std::list<MemBlock> memMap;

    public:
        MemAllocator(Addr deviceAddr) {
            baseAddr = deviceAddr;
            MemBlock initBlock = {
                .isValid = true,
                .vaddr = AddrRange(0, 0),
                .paddr = AddrRange(baseAddr, baseAddr)
            };
            memMap.emplace_front(initBlock);
        }
        ~MemAllocator(){}

        /* Alloc a block of memory
         * Return a virtual addr
         */
        MemBlock allocMem(size_t size);

        void recycleMem();

        /* Get MemBlock from physical addr
         */
        MemBlock* getPhyBlock(Addr paddr);

        /* Get MemBlock from virtual addr
         */
        MemBlock* getVirBlock(Addr vaddr);

        /* Get physical addr from virtual addr
         */
        Addr getPhyAddr(Addr vaddr);

        /* Get virtual addr from physical addr
         */
        Addr getVirAddr(Addr paddr);

        uint64_t getSize();
};

#endif /* __ACCEL_DEFS_HH__ */
