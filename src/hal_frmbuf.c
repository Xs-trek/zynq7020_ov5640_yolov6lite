/* hal_frmbuf.c — v_frmbuf_wr V2.4 控制
 *
 * 帧同步: 单帧模式，轮询 CTRL bit1 (ap_done, R/COR)
 */
#include "hal_frmbuf.h"
#include "hal_mmio.h"
#include "config.h"
#include <stdio.h>
#include <unistd.h>

static volatile uint32_t *rst_gpio = NULL;
static volatile uint32_t *fbwr     = NULL;

static const uint32_t fb_phys[FRAME_BUF_CNT] = {
    FRAME_BUF_0, FRAME_BUF_1, FRAME_BUF_2
};

int frmbuf_init(void)
{
    /* 防止重复初始化 */
    if (fbwr != NULL) {
        fprintf(stderr, "[frmbuf] already initialized\n");
        return 0;
    }

    /* 映射复位 GPIO */
    rst_gpio = (volatile uint32_t *)mmio_map(
                   FRMBUF_RST_GPIO_BASE, 0x1000);
    if (!rst_gpio) {
        fprintf(stderr, "[frmbuf] failed to map rst_gpio\n");
        return -1;
    }

    /* 先拉低复位，再释放 */
    reg_wr(rst_gpio, 0x00, FRMBUF_GPIO_RESET);
    usleep(10000);
    reg_wr(rst_gpio, 0x00, FRMBUF_GPIO_RELEASE);
    usleep(10000);
    printf("[frmbuf] Reset released, GPIO=0x%08X\n",
           reg_rd(rst_gpio, 0x00));

    /* 映射 frmbuf_wr */
    fbwr = (volatile uint32_t *)mmio_map(FRMBUF_WR_BASE, 0x1000);
    if (!fbwr) {
        fprintf(stderr, "[frmbuf] failed to map frmbuf_wr\n");
        mmio_unmap(rst_gpio, 0x1000);
        rst_gpio = NULL;
        return -1;
    }

    /* 确保 IP 停止 */
    reg_wr(fbwr, FRMBUF_CTRL, 0x00);
    usleep(10000);

    /* 配置参数 */
    reg_wr(fbwr, FRMBUF_WIDTH,     IMG_W);
    reg_wr(fbwr, FRMBUF_HEIGHT,    IMG_H);
    reg_wr(fbwr, FRMBUF_STRIDE,    IMG_W * IMG_BPP);
    reg_wr(fbwr, FRMBUF_FMT,       FRMBUF_FMT_RGB8);
    reg_wr(fbwr, FRMBUF_ADDR,      fb_phys[0]);
    reg_wr(fbwr, FRMBUF_ADDR_HIGH, 0x00);

    printf("[frmbuf] Configured: %dx%d stride=%d fmt=%d\n",
           reg_rd(fbwr, FRMBUF_WIDTH),
           reg_rd(fbwr, FRMBUF_HEIGHT),
           reg_rd(fbwr, FRMBUF_STRIDE),
           reg_rd(fbwr, FRMBUF_FMT));
    return 0;
}

int frmbuf_start_frame(int buf_idx)
{
    if (!fbwr) {
        fprintf(stderr, "[frmbuf] not initialized\n");
        return -1;
    }
    if (buf_idx < 0 || buf_idx >= FRAME_BUF_CNT) {
        fprintf(stderr, "[frmbuf] invalid buf_idx %d\n", buf_idx);
        return -1;
    }
    reg_wr(fbwr, FRMBUF_ADDR, fb_phys[buf_idx]);
    reg_wr(fbwr, FRMBUF_CTRL, 0x01);  /* ap_start=1 */
    return 0;
}

int frmbuf_wait_done(int timeout_ms)
{
    if (!fbwr) {
        fprintf(stderr, "[frmbuf] not initialized\n");
        return -1;
    }
    if (timeout_ms <= 0) timeout_ms = 2000;

    for (int waited = 0; waited < timeout_ms; waited++) {
        uint32_t ctrl = reg_rd(fbwr, FRMBUF_CTRL);
        if (ctrl & 0x02) return 0;  /* bit1=ap_done (COR) */
        usleep(1000);
    }

    fprintf(stderr, "[frmbuf] Timeout after %dms! CTRL=0x%08X\n",
            timeout_ms, reg_rd(fbwr, FRMBUF_CTRL));
    return -1;
}

void frmbuf_stop(void)
{
    if (fbwr)
        reg_wr(fbwr, FRMBUF_CTRL, 0x00);
}

void frmbuf_close(void)
{
    frmbuf_stop();
    if (fbwr)     { mmio_unmap(fbwr, 0x1000);     fbwr = NULL; }
    if (rst_gpio) { mmio_unmap(rst_gpio, 0x1000); rst_gpio = NULL; }
}
