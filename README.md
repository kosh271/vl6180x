# vl6180x
Linux driver for the vl6180x.  Uses IIO subsystem (industrial I/O).

## Linux Kernel Version
The Industrial I/O subsystem was introduced into the Linux Kernel in the early 3.x versions.
This driver was tested using Kernel 4.9.77

## Other notes
- This driver isn't what I would consider polished.  
- While it does function, there is much room for improvement.
- If you see a problem with the driver, feel free to let me know.

## Specific items needing updates
- ALS (Ambient Light Sensor) does not use the ready interrupt, hard delay coded instead
- There is no configuration available at runtime, only compile time (probably won't ever be)

## Test environment
Testing for this driver done on a Raspberry PI.

If you are not loading with insmod instead of modprobe, you will need to load the iio module manually using: "sudo modprobe industrialio"