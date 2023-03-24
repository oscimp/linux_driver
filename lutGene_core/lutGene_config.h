#ifndef LUTGENE_CONFIG_H
#define LUTGENE_CONFIG_H

#define LUTGENE_RAM_LENGTH 0
#define LUTGENE_PRESCALER  1
#define LUTGENE_ENABLE     2
#define LUTGENE_TYPE       3

/* ioctl */
#define LUTGENE_SET(__reg)_IOW('a', __reg, unsigned long)
#define LUTGENE_GET(__reg)_IOR('a', __reg, unsigned long)

#endif /* LUTGENE_CONFIG_H */
