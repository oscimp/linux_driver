#ifndef EDFB_CONFIG_H
#define EDFB_CONFIG_H

/* platform device */
struct plat_edfb_port {
	const char *name;	/* instance name */
	int num;		/* instance number */
	int idnum;		/* identity number */
	int idoffset;		/* identity relative address */
	struct edfb_dev *sdev;/* struct for main device structure */
};

/* ioctl */
#define EDFB_POINT_POS	(0x01<<2)
#define EDFB_SET_POINT_POS _IOW('a', EDFB_POINT_POS, char)
#define EDFB_GET_POINT_POS _IOR('a', EDFB_POINT_POS, char)

#endif /* EDFB_CONFIG_H */

