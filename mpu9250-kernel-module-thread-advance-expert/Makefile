# Makefile for MPU9250 driver and tests
KDIR ?= /lib/modules/$(shell uname -r)/build
CC = gcc
CROSS_COMPILE ?=
ARCH ?= arm64
VALGRIND = valgrind
DOXYGEN = doxygen

# Kernel module
obj-m += mpu9250_driver.o mpu9250_fileops.o mpu9250_ops.o mpu9250_timer.o custom_rwlock.o weak_semaphore.o

# User-space test programs
USER_PROGS = user_space_test mpu9250_ipc_test map_reduce_test bridge_problem_test assembly_line_test vaccination_drive_test

# Default target
all: kernel_module user_progs dtb

# Build kernel module
kernel_module:
	$(MAKE) -C $(KDIR) M=$(CURDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

# Build user-space programs
user_progs: $(USER_PROGS)

user_space_test: user_space_test.c mpu9250.h
	$(CC) -o user_space_test user_space_test.c -pthread

mpu9250_ipc_test: mpu9250_ipc_test.c mpu9250.h
	$(CC) -o mpu9250_ipc_test mpu9250_ipc_test.c -pthread -lrt

map_reduce_test: map_reduce_test.c mpu9250.h
	$(CC) -o map_reduce_test map_reduce_test.c -pthread

bridge_problem_test: bridge_problem_test.c
	$(CC) -o bridge_problem_test bridge_problem_test.c -pthread

assembly_line_test: assembly_line_test.c mpu9250.h
	$(CC) -o assembly_line_test assembly_line_test.c -pthread

vaccination_drive_test: vaccination_drive_test.c
	$(CC) -o vaccination_drive_test vaccination_drive_test.c -pthread

# Build device tree overlay
dtb: mpu9250.dtb

mpu9250.dtb: mpu9250.dts
	dtc -@ -I dts -O dtb -o mpu9250.dtb mpu9250.dts

# Install kernel module and device tree
install: kernel_module dtb
	$(MAKE) -C $(KDIR) M=$(CURDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules_install
	cp mpu9250.dtb /boot/overlays/
	depmod -a

# Uninstall kernel module and device tree
uninstall:
	rm -f /lib/modules/$(shell uname -r)/extra/mpu9250_*.ko
	rm -f /boot/overlays/mpu9250.dtb
	depmod -a

# Clean build artifacts
clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
	rm -f $(USER_PROGS) mpu9250.dtb
	rm -rf doc

# Check dependencies extended for threading
check_deps:
	@which $(CC) || (echo "Error: $(CC) not found"; exit 1)
	@which dtc || (echo "Error: dtc not found"; exit 1)
	@which $(VALGRIND) || (echo "Warning: valgrind not found, concurrency tests may fail")
	@ldconfig -p | grep libpthread || (echo "Error: pthread library not found"; exit 1)
	@[ -d $(KDIR) ] || (echo "Error: Kernel directory $(KDIR) not found"; exit 1)
	@ls /usr/include/linux/kthread.h || (echo "Error: Kernel threading support missing"; exit 1)

# Generate Doxygen documentation with thread notes
docs:
	$(DOXYGEN) Doxyfile

# Concurrency testing with valgrind/helgrind
thread_test: user_progs
	$(VALGRIND) --tool=helgrind ./user_space_test --accel --gyro --mag --dmp
	$(VALGRIND) --tool=helgrind ./mpu9250_ipc_test --mq --shm --fifo --pipe
	$(VALGRIND) --tool=helgrind ./map_reduce_test
	$(VALGRIND) --tool=helgrind ./bridge_problem_test
	$(VALGRIND) --tool=helgrind ./assembly_line_test
	$(VALGRIND) --tool=helgrind ./vaccination_drive_test

.PHONY: all kernel_module user_progs dtb install uninstall clean check_deps docs thread_test