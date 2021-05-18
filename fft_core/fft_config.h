#ifndef FFT_CONFIG_H
#define FFT_CONFIG_H

/* platform device */
struct plat_fft_port {
	const char *name;	/* instance name */
	int num;		/* instance number */
	int idnum;		/* identity number */
	int idoffset;		/* identity relative address */
	struct fft_dev *sdev;/* struct for main device structure */
};

#endif /* FFT_CONFIG_H */

