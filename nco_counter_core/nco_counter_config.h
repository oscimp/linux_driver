#ifndef NCO_COUNTER_CONFIG_H
#define NCO_COUNTER_CONFIG_H

/* platform device */
struct plat_nco_counter_port {
	const char *name;	/* instance name */
	int num;		/* instance number */
	int idnum;		/* identity number */
	int idoffset;		/* identity relative address */
	struct nco_counter_dev *sdev;/* struct for main device structure */
};

/* ioctl */
#define REG_ID 			0x00
#define REG_RST_ACCUM 	0x00
#define REG_POFF 		0x01
#define REG_CTRL 		0x02
#define REG_PINC 		0x03
#define REG_MAX_ACCUM 	0x04
/* specific */
#define REG_PINC_L 		0x03
#define REG_PINC_H 		0x04
#define REG_MAX_ACCUM_L 0x05
#define REG_MAX_ACCUM_H 0x06

#define CTRL_PINC_SW (1 << 0)
#define CTRL_POFF_SW (1 << 1)
#define NCO_COUNTER_SET(__reg__) _IOW('a', __reg__, uint64_t)
#define NCO_COUNTER_GET(__reg__) _IOR('a', __reg__, uint64_t)

#endif /* NCO_COUNTER_CONFIG_H */

