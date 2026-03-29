#ifndef HAL_MMIO_H
#define HAL_MMIO_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

/* 打开 /dev/mem，返回 fd，失败返回 -1 */
int mmio_init(void);

/* 关闭 /dev/mem */
void mmio_close(void);

/* mmap 物理地址，返回虚拟地址，失败返回 NULL */
volatile void *mmio_map(off_t phys_addr, size_t size);

/* 取消映射 */
void mmio_unmap(volatile void *vaddr, size_t size);

/* 寄存器读写 */
static inline void reg_wr(volatile uint32_t *base, unsigned offset, uint32_t val)
{
    base[offset / 4] = val;
}

static inline uint32_t reg_rd(volatile uint32_t *base, unsigned offset)
{
    return base[offset / 4];
}

#endif
