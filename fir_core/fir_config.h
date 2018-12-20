#ifndef FIR_CONFIG_H
#define FIR_CONFIG_H

/* platform device */
struct plat_fir_port {
	const char *name;	/* instance name */
	int num;		/* instance number */
	int idnum;		/* identity number */
	int idoffset;		/* identity relative address */
	struct fir_dev *sdev;/* struct for main device structure */
};

#endif /* FIR_CONFIG_H */

