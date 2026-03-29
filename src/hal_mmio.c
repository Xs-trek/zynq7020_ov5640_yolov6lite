/* hal_mmio.c — 物理地址映射层
 *
 * 说明: 使用 /dev/mem + mmap 直接访问 PL 寄存器
 * 适用: Zynq PL 自定义 IP 控制，延迟敏感路径
 * 约束: mmio_init/mmio_close 非线程安全，须在主线程调用
 */
#include "hal_mmio.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

static int mem_fd = -1;

int mmio_init(void)
{
    if (mem_fd >= 0) return 0;

    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("[mmio] open /dev/mem");
        return -1;
    }
    return 0;
}

void mmio_close(void)
{
    if (mem_fd >= 0) {
        close(mem_fd);
        mem_fd = -1;
    }
}

volatile void *mmio_map(off_t phys_addr, size_t size)
{
    if (mem_fd < 0) {
        fprintf(stderr, "[mmio] not initialized\n");
        return NULL;
    }
    /* /dev/mem 要求地址 4KB 页对齐 */
    if (phys_addr & 0xFFF) {
        fprintf(stderr, "[mmio] addr 0x%lx not page-aligned\n",
                (unsigned long)phys_addr);
        return NULL;
    }
    void *p = mmap(NULL, size, PROT_READ | PROT_WRITE,
                   MAP_SHARED, mem_fd, phys_addr);
    if (p == MAP_FAILED) {
        perror("[mmio] mmap");
        return NULL;
    }
    return (volatile void *)p;
}

void mmio_unmap(volatile void *vaddr, size_t size)
{
    if (vaddr) {
        munmap((void *)vaddr, size);
    }
}
