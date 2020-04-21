# linux_driver
OscillatorIMP ecosystem Linux drivers. These drivers are designed to help the developer by complying with the usual operating system abstraction hierarchy of making the kernel exchange with hardware and transfering data between kernel space and userspace. While direct access to IP registers is possible (and sometimes necessary if no kernel module has been written for  given IP -- using using the devmem application provided by Busybox in Buildroot -- following the addressing described at https://github.com/oscimp/oscimpDigital/tree/master/doc/IP), we encourage using the abstraction layers hiding the low level access to hardware to the userspace programmer.

Make sure the buildroot toolchain path is included in your $PATH. If not,
```
export PATH=/somewhere/output/host/usr/bin:$PATH
```
where ``somewhere`` is the location of the buildroot, installed as described at
https://github.com/trabucayre/redpitaya

In this directory,
```
make
```
compiles all the linux kernel modules for the Redpitaya board. These modules will be sent to
the Redpita and relevant modules should be inserted there using 
```
insmod mymodule.ko
```
for inserting the module named ```mymodule```. Alternatively, each project should include a
```
projectname_us.sh
```
where "_us" stands for userspace which takes care of all the necessary steps, including loading the bitstream and the 
necessary kernel modules. See for example https://github.com/oscimp/oscimpDigital/blob/master/doc/tutorials/redpitaya/3-PLPS/app/tutorial3_us.sh which loads the data_to_ram kernel module assumed to be located at CORE_MODULES_DIR, here defined
as ../../modules if all modules have been copied to the Redpitaya.
