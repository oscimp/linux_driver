#ifndef PIDV3_AXI_CONFIG_H
#define PIDV3_AXI_CONFIG_H

#define REG_PIDV3_AXI_KP        0
#define REG_PIDV3_AXI_KI        1
#define REG_PIDV3_AXI_KD        2
#define REG_PIDV3_AXI_SETPOINT  3
#define REG_PIDV3_AXI_SIGN      4
#define REG_PIDV3_AXI_INT_RST   5
#define REG_PIDV3_AXI_INPUT     6

/* reg input bit */
#define PIDV3_AXI_SETPOINT_SW   (0x1 << 0)
#define PIDV3_AXI_SETPOINT_HW   (0x2 << 0)
#define PIDV3_AXI_KP_SW         (0x1 << 2)
#define PIDV3_AXI_KP_HW         (0x2 << 2)
#define PIDV3_AXI_KI_SW         (0x1 << 4)
#define PIDV3_AXI_KI_HW         (0x2 << 4)
#define PIDV3_AXI_KD_SW         (0x1 << 6)
#define PIDV3_AXI_KD_HW         (0x2 << 6)
#define PIDV3_AXI_SIGN_SW       (0x1 << 8)
#define PIDV3_AXI_SIGN_HW       (0x2 << 8)
#define PIDV3_AXI_INT_RST_SW    (0x1 << 10)
#define PIDV3_AXI_INT_RST_HW    (0x2 << 10)

/* ioctl */
#define PIDV3_AXI_SET(__reg)_IOW('a', __reg, unsigned long)
#define PIDV3_AXI_GET(__reg)_IOR('a', __reg, unsigned long)

#endif /* PIDV3_AXI_CONFIG_H */

