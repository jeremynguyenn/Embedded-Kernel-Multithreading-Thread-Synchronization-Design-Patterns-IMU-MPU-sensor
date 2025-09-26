# MPU9250 Kernel Driver for Raspberry Pi (RUN follow the file mpu9250-kernel-module-thread-advance-update)

This project provides a Linux kernel driver for the **MPU9250**, a 9-axis MotionTracking device from Invensense (now TDK), tailored for Raspberry Pi. The MPU9250 integrates a 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer, ideal for motion sensing, orientation detection, and navigation in embedded systems like drones, wearables, and IoT devices. The driver leverages I2C communication and incorporates advanced concurrency, synchronization, file operations, system calls, library functions, blocking/non-blocking calls, atomic operations, race condition handling, user/kernel mode process management, POSIX threads, thread synchronization, inter-process communication (IPC), memory management, and signal handling.

## Table of Contents
- [Introduction to the Sensor](#introduction-to-the-sensor)
- [Features](#features)
- [Code Structure](#code-structure)
- [Build](#build)
- [Install](#install)
- [Usage](#usage)
- [Expected Results](#expected-results)
- [License](#license)

## Introduction to the Sensor

The **MPU9250** is a low-power, high-performance Inertial Measurement Unit (IMU) with:
- **3-Axis Accelerometer**: Measures linear acceleration (X, Y, Z) with configurable ranges (±2g, ±4g, ±8g, ±16g).
- **3-Axis Gyroscope**: Measures angular velocity with ranges (±250°/s, ±500°/s, ±1000°/s, ±2000°/s).
- **3-Axis Magnetometer (AK8963)**: Measures magnetic fields for compass/orientation data.
- **Digital Motion Processor (DMP)**: On-chip processing for quaternions, gesture recognition, and sensor fusion.
- **FIFO Buffer**: Hardware buffer for efficient data collection.
- **Interrupts**: Supports data-ready, FIFO overflow, and DMP interrupts.

The sensor communicates via I2C (default address 0x68) and is widely used in robotics, VR/AR, and stabilization systems. This driver exposes functionality through `/dev/mpu9250`, sysfs attributes, and the Linux input subsystem.

## Features

- **Kernel Integration**: I2C-based probe/remove, power management, IRQ handling with atomic operations and race condition prevention.
- **Data Access**: Read accelerometer, gyroscope, magnetometer, and DMP quaternion data via file operations (read/write/ioctl/poll/mmap) with blocking/non-blocking support.
- **Concurrency**: Thread pools for parallel sensor reads, dynamic thread count, pause/resume, using POSIX threads (user space) and kthreads (kernel space).
- **Synchronization**: Custom recursive mutexes, read-write locks, condition variables, semaphores, barriers, notifiers for Inter-Thread Communication (ITC), dining philosophers pattern, and deadlock detection/prevention.
- **User-Space Tests**: Multi-threaded tests for sensor data and IPC (pipes, FIFO, POSIX message queues, semaphores, shared memory), with signal handling, detached/joinable threads, and publisher-subscriber model.
- **Device Tree Overlay**: Configurable via Device Tree Source (DTS) for I2C, interrupts, and threading parameters.
- **Build and Documentation**: Makefile for building with GNU-GCC, Doxygen support, and valgrind/helgrind for concurrency validation.
- **Process Management**: Fork() system call, child-parent process handling, command line argument parsing, memory layout (code, data, stack, heap).
- **Memory Management**: Kernel allocations (vmalloc, kzalloc), user-space malloc/mmap, and memory segment management.
- **Signals**: Signal handlers, sending signals, and default signal behaviors.

## UML Diagram
- **Class Diagram:**
<img width="1604" height="1509" alt="image" src="https://github.com/user-attachments/assets/4d656f46-ba31-4d65-b768-aeafbdde79e6" />


- **Component Diagram**
<img width="6157" height="4378" alt="image" src="https://github.com/user-attachments/assets/57cf3fb8-a0e9-4bbe-83e0-3b09cdc34b82" />

<img width="2970" height="346" alt="image" src="https://github.com/user-attachments/assets/41a6eb60-426f-4a03-b396-5e2f273461b5" />


- **Sequence Diagram**
<img width="1483" height="818" alt="image" src="https://github.com/user-attachments/assets/57271c44-e970-4c73-a26a-398f5d2a1346" />

## Code Structure

The project consists of 8 files: 4 kernel driver files, 1 header, 2 user-space test programs, 1 Device Tree overlay, and 1 Makefile. Below is a breakdown of each file, covering file operations, system calls, library functions, compiling with GNU-GCC, blocking/non-blocking calls, atomic operations, race conditions, user/kernel mode process management, POSIX threads, thread synchronization, IPC, memory management, and signals.

### mpu9250.h
- **Purpose**: Defines constants, structs, enums, IOCTL commands, and function prototypes.
- **Key Features**:
  - Structs: `mpu9250_data` (thread_pool, rwlock_t, semaphore, atomic_t), `mpu9250_mq_data` (accel/gyro/mag/quat).
  - Enums: Scales, interrupts.
  - IOCTLs: Sensor reads, scale setting, calibration, thread control.
  - Custom recursive mutex with atomic_t.
- **Knowledge Used**:
  - **File Operations/System Calls**: IOCTL definitions.
  - **Library Functions**: Kernel memcpy, memset.
  - **Atomic Operations**: atomic_t for deadlock detection.
  - **Race Conditions**: Mutex, rwlock for synchronization.
  - **Thread Synchronization**: recursive_mutex, rwlock_t, notifier_head.
  - **Process Management**: Kthread structs.
  - **Memory Management**: Data arrays (vmalloc).

### mpu9250_ops.c
- **Purpose**: Implements sensor operations: I2C read/write, initialization, FIFO/DMP setup, calibration.
- **Key Features**:
  - I2C regmap communication.
  - FIFO/DMP initialization.
  - Calibration with dining philosophers (semaphores) and cleanup handlers.
- **Knowledge Used**:
  - **File Operations/System Calls**: Regmap read/write.
  - **Library Functions**: Kernel memcpy, dev_err.
  - **Blocking Calls**: wait_event_interruptible.
  - **Atomic Operations**: atomic_t for counters.
  - **Race Conditions**: Semaphores, monitors.
  - **Thread Synchronization**: wait_queue, semaphores.
  - **Process Management**: Kthread cancellation.
  - **Memory Management**: vmalloc for buffers.

### mpu9250_driver.c
- **Purpose**: Core driver logic for probe/remove, sysfs attributes, thread pool, power management.
- **Key Features**:
  - I2C device probing.
  - Sysfs attrs with rwlocks.
  - Thread pool with ITC via notifiers.
- **Knowledge Used**:
  - **File Operations/System Calls**: Sysfs, IRQ registration.
  - **Library Functions**: devm_*, dev_err.
  - **Blocking/Non-blocking**: wait_queue.
  - **Atomic Operations**: atomic_inc/dec.
  - **Race Conditions**: Lock ordering, try_lock.
  - **Thread Synchronization**: Barriers, wait-queues, notifiers.
  - **Process Management**: kthread_create/stop.
  - **Memory Management**: kzalloc, vmalloc.

### mpu9250_fileops.c
- **Purpose**: File operations for `/dev/mpu9250` (open/release, read/write, ioctl, poll, mmap).
- **Key Features**:
  - Blocking (wait_event) and non-blocking (poll, O_NONBLOCK) support.
  - Ioctl for sensor/thread control.
  - Mmap for FIFO buffer sharing.
- **Knowledge Used**:
  - **File Operations/System Calls**: open, read, write, ioctl, poll, mmap.
  - **Library Functions**: copy_to/from_user.
  - **Blocking/Non-blocking**: wait_event, poll_wait.
  - **Atomic Operations**: atomic_t flags.
  - **Race Conditions**: Recursive mutex, rwlock, spinlock.
  - **Thread Synchronization**: wait_queue, recursive_mutex.
  - **Memory Management**: remap_vmalloc_range.

### mpu9250.dts
- **Purpose**: Device Tree overlay for MPU9250 on I2C bus.
- **Key Features**:
  - Enables I2C1, MPU9250 node (address, interrupt, magnetometer).
  - Configures threading parameters.
- **Knowledge Used**:
  - Device Tree syntax for hardware configuration.

### mpu9250_ipc_test.c
- **Purpose**: User-space test for IPC mechanisms with threading.
- **Key Features**:
  - Threads for message queue, shared memory, FIFO, pipe.
  - Detached/joinable threads, command line arguments.
- **Knowledge Used**:
  - **File Operations/System Calls**: mq_open, shm_open, pipe, mkfifo.
  - **Library Functions**: malloc, printf, getopt_long.
  - **Blocking/Non-blocking**: sem_timedwait, non-blocking read.
  - **Atomic Operations**: POSIX mutex/semaphore.
  - **Race Conditions**: pthread_mutex_t, pthread_rwlock_t, pthread_barrier_t.
  - **POSIX Threads**: pthread_create, pthread_exit, pthread_join, pthread_detach, pthread_self.
  - **Thread Synchronization**: mutex, condition variables, rwlocks.
  - **IPC**: Message queues, shared memory, semaphores, FIFO, pipes.
  - **Process Management**: fork(), command line args.
  - **Signals**: Handlers for SIGINT/TERM/SEGV, kill(SIGTERM).
  - **Memory Management**: malloc/free, mmap.

### user_space_test.c
- **Purpose**: User-space test for sensor reading with multi-threading.
- **Key Features**:
  - Threads for accel, gyro, mag, DMP reads.
  - Detached/joinable threads, command line arguments.
- **Knowledge Used**:
  - **File Operations/System Calls**: open, write, ioctl.
  - **Library Functions**: malloc, printf, getopt_long.
  - **Blocking Calls**: pthread_cond_wait, read/ioctl.
  - **Race Conditions**: pthread_mutex_t, pthread_rwlock_t, pthread_barrier_t.
  - **POSIX Threads**: pthread_create, pthread_exit, pthread_join, pthread_detach, pthread_self.
  - **Thread Synchronization**: mutex, condition variables, rwlocks.
  - **Process Management**: fork(), execl, command line args.
  - **Signals**: Handlers for SIGINT/TERM/USR1.
  - **Memory Management**: malloc/free, thread stacks.

### Makefile
- **Purpose**: Build script for kernel module, user-space tests, DTB, documentation.
- **Key Features**:
  - Compiles kernel module, user programs with GNU-GCC.
  - Concurrency tests with valgrind/helgrind.
- **Knowledge Used**:
  - **Compiling with GNU-GCC**: gcc with -pthread, -lrt.
  - **Library Functions**: Links pthread, rt.
  - **Process Management**: Supports cross-compilation.

## Build

To build all artifacts (kernel module, user-space tests, device tree overlay), run:

```bash
make all
```

To build specific components:
- Kernel module: `make kernel_module`
- Device tree overlay: `make dtb`
- User-space tests: `make app`

File run test DEMO: `user_space_test.c`, `mpu9250_ipc_test.c`.

To run concurrency tests with valgrind/helgrind:

```bash
make thread_test
```

To generate Doxygen documentation:

```bash
make docs
```

To delete all files except artifacts and sources:

```bash
make clean
```

To delete all files except sources:

```bash
make cleanall
```

## Install

To install the “mpu9250_driver” for the Raspberry Pi, follow these steps. Ensure kernel headers are installed (`sudo apt install raspberrypi-kernel-headers`).

### Step 1: Navigate to the /boot Directory
```bash
cd /boot
```

### Step 2: Convert Device Tree Blob to Source
Convert the Raspberry Pi device tree blob (.dtb) to a device tree source (.dts) file:

```bash
dtc -I dtb -O dts -o bcm2711-rpi-4-b.dts bcm2711-rpi-4-b.dtb
```

**Note**: Replace `bcm2711-rpi-4-b.dtb` with the .dtb file for your Raspberry Pi model (e.g., `bcm2710-rpi-3-b.dtb` for Raspberry Pi 3).

### Step 3: Edit the Device Tree Source
Open the .dts file (e.g., with `nano bcm2711-rpi-4-b.dts`). Locate the I2C-1 section (under `&i2c1`). Add the MPU9250 configuration from `mpu9250.dts`:
- Copy `fragment@0` (I2C1 with mpu9250@68, AK8963 magnetometer).
- Copy `fragment@1` (GPIO interrupt pin).
- Copy `fragment@2` (pinctrl configuration).
- Ensure `status = "okay";` for I2C1.

### Step 4: Recompile and Reboot
Save changes, recompile the .dts file back to .dtb, and reboot:

```bash
dtc -I dts -O dtb -o bcm2711-rpi-4-b.dtb bcm2711-rpi-4-b.dts
sudo reboot
```

**Note**: Replace filenames for your Raspberry Pi model.

### Step 5: Build and Install the Driver
1. Ensure all source files (`mpu9250_driver.c`, `mpu9250_fileops.c`, `mpu9250_ops.c`, `mpu9250.h`, `Makefile`, `mpu9250.dts`) are in the same directory.
2. Build the driver:
   ```bash
   make
   ```
3. Install the kernel module:
   ```bash
   sudo insmod mpu9250_driver.ko
   ```
4. Apply the device tree overlay (if not integrated in Step 3):
   ```bash
   sudo dtoverlay mpu9250.dtb
   ```
5. Check installation status:
   ```bash
   dmesg | grep mpu9250
   ```
   Expected output: `MPU9250 probed successfully, version 2.3`.
6. To remove the module:
   ```bash
   sudo rmmod mpu9250_driver
   ```
7. To remove the device tree overlay:
   ```bash
   sudo dtoverlay -r mpu9250
   ```

**Note**: Ensure I2C is enabled in `/boot/config.txt` with `dtparam=i2c_arm=on`. Connect MPU9250 to I2C pins (GPIO 2/3 for SDA/SCL) and interrupt pin to GPIO 17.

## Usage

The driver creates `/dev/mpu9250` for user-space access, supporting file operations, system calls, POSIX threads, and more. Below are steps to interact with the sensor:

1. **Open Device** (System Call: open):
   ```c
   int fd = open("/dev/mpu9250", O_RDWR);
   if (fd < 0) perror("Failed to open /dev/mpu9250");
   ```

2. **Configure Scales and Sample Rate** (System Call: ioctl):
   ```c
   ioctl(fd, MPU9250_IOCTL_SET_ACCEL_SCALE, ACCEL_SCALE_2G);
   ioctl(fd, MPU9250_IOCTL_SET_GYRO_SCALE, GYRO_SCALE_250DPS);
   ioctl(fd, MPU9250_IOCTL_SET_SAMPLE_RATE, 100);
   ```

3. **Read Sensor Data** (System Calls: ioctl, read):
   ```c
   struct mpu9250_sensor_data data;
   ioctl(fd, MPU9250_IOCTL_READ_ACCEL, &data);
   printf("Accel: %.2f %.2f %.2f g\n", data.values[0], data.values[1], data.values[2]);
   ```
   Use ioctls for gyro, magnetometer, DMP. Blocking read uses `read(fd, buf, len)`.

4. **Thread Control** (System Call: ioctl):
   ```c
   ioctl(fd, MPU9250_IOCTL_PAUSE_THREADS);
   ioctl(fd, MPU9250_IOCTL_SET_NUM_THREADS, 4);
   ioctl(fd, MPU9250_IOCTL_RESUME_THREADS);
   ```

5. **Run Tests** (Process Management, Signals, IPC, Threads):
   ```bash
   ./user_space_test --accel --gyro --mag --dmp --sample-rate 100 --detach
   ./mpu9250_ipc_test --mq --shm --fifo --pipe --detach --queue-size 10
   ```
   Tests use fork(), signals (SIGINT/TERM/USR1), POSIX threads, IPC (pipes, FIFO, message queues, semaphores, shared memory), and synchronization (mutex, condition variables, rwlocks).

6. **Memory Mapping** (System Call: mmap):
   ```c
   void *fifo_map = mmap(NULL, MPU9250_MAX_FIFO, PROT_READ, MAP_SHARED, fd, 0);
   munmap(fifo_map, MPU9250_MAX_FIFO);
   ```

## Expected Results

With the MPU9250 at rest on a flat surface:
- **Accelerometer**: ~[0.00, 0.00, 1.00] g (±0.05g noise after calibration).
- **Gyroscope**: ~[0.00, 0.00, 0.00] °/s (±0.1°/s noise).
- **Magnetometer**: Varies, e.g., [X, Y, Z] in μT (~25-65 μT).
- **Quaternions (DMP)**: ~[1.00, 0.00, 0.00, 0.00].

Example output:
```
Callback: Accel: 0.02 0.01 0.98, Gyro: 0.05 -0.03 0.02, Mag: 30.50 42.10 -48.20, Quat: 1.00 0.00 0.00 0.00
```

**Notes**:
- Calibration (`MPU9250_IOCTL_CALIBRATE`) uses dining philosophers for synchronization.
- Errors like `-EIO` indicate I2C issues (check wiring/address).
- Concurrency tests (valgrind/helgrind) confirm no races/deadlocks.

## License

Licensed under the **GPL** (see `mpu9250_driver.c`). Author: Nguyen Nhan. Version: 2.3.
