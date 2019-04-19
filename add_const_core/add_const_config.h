#ifndef ADD_CONST_CONFIG_H
#define ADD_CONST_CONFIG_H

/* platform device */
struct plat_add_const_port {
	const char *name;	/* instance name */
	int num;		/* instance number */
	int idnum;		/* identity number */
	int idoffset;		/* identity relative address */
	struct add_const_dev *sdev;/* struct for main device structure */
};

/* ioctl */
#define ADD_CONST_SET_CONST _IOW('a', 0, long long)
#define ADD_CONST_GET_CONST _IOR('a', 0, long long)

#endif /* ADD_CONST_CONFIG_H */

