#ifndef AXI_TO_DAC_CONFIG_H
#define AXI_TO_DAC_CONFIG_H

/* platform device */
struct plat_axi_to_dac_port {
	const char *name;	/* instance name */
	int num;		/* instance number */
	int idnum;		/* identity number */
	int idoffset;		/* identity relative address */
	struct axi_to_dac_dev *sdev;/* struct for main device structure */
};

#define AXI_TO_DAC_ID        0
#define AXI_TO_DAC_DATA_A    1
#define AXI_TO_DAC_DATA_B    2
#define AXI_TO_DAC_EN_HIGH   3
#define AXI_TO_DAC_SYNC_CHAN 4

/* reg conf bit */
#define AXI_TO_DAC_SYNC_EN       (0x1 << 0)
#define AXI_TO_DAC_DATAA_EN_HIGH (0x1 << 1)
#define AXI_TO_DAC_DATAB_EN_HIGH (0x2 << 1)
#define AXI_TO_DAC_BOTH_EN_HIGH  (0x3 << 1)

/* ioctl */
#define AXI_TO_DAC_SET(__reg)_IOW('a', __reg, unsigned long)
#define AXI_TO_DAC_GET(__reg)_IOR('a', __reg, unsigned long)

#endif /* AXI_TO_DAC_CONFIG_H */

