#ifndef DATA_TO_RAM_CONFIG_H
#define DATA_TO_RAM_CONFIG_H

struct data_to_ram_data {
	int *datai;
	int *dataq;
};

/* platform device */
struct plat_data_to_ram_port {
	const char *name;	/* instance name */
	int num;		/* instance number */
	int idnum;		/* identity number */
	int idoffset;		/* identity relative address */
	struct data_to_ram_dev *sdev;	/* struct for main device structure */
};

#define DCTR_START 1
#define DCTR_IOCTL _IOW('a', DCTR_START, char)

#endif				/* DATA_TO_RAM_CONFIG_H */
