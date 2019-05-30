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

/* ioctl */
#define AXI_TO_DAC_SET_DACA _IOW('a', 0, unsigned long)
#define AXI_TO_DAC_SET_DACB _IOW('a', 1, unsigned long)

#endif /* AXI_TO_DAC_CONFIG_H */

