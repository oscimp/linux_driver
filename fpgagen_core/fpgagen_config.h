#ifndef FPGAGEN_CONFIG_H
#define FPGAGEN_CONFIG_H

/* platform device */
struct plat_fpgagen_port {
	const char *name;	/* instance name */
	int num;		/* instance number */
	int idnum;		/* identity number */
	int idoffset;		/* identity relative address */
	struct fpgagen_dev *sdev;/* struct for main device structure */
};

/* ioctl */
#define FPGAGEN_SET_REGISTER(__reg) _IOW('a', __reg, unsigned long)
#define FPGAGEN_GET_REGISTER(__reg) _IOR('a', __reg, unsigned long)

#endif /* FPGAGEN_CONFIG_H */

