# linux_driver
OscillatorIMP ecosystem Linux drivers

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
