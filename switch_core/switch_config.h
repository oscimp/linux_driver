#ifndef SWITCH_CONFIG_H
#define SWITCH_CONFIG_H

// IOTCL
#define IOC_MAGIC 'a'	
#define SWITCH_SELECT_INPUT _IOW(IOC_MAGIC, 0, int)

/* platform device */
struct plat_switch_port {
	const char *name;
	int num;
	int idnum;
	int idoffset;
	struct switch_dev *sdev;
};
#endif
