# MPU9250 Kernel Driver for Raspberry Pi

This project provides a Linux kernel driver for the **MPU9250**, a 9-axis MotionTracking device from Invensense (now TDK). The MPU9250 integrates a 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer, making it ideal for applications like motion sensing, orientation detection, and navigation in embedded systems such as drones, wearables, and IoT devices. The driver is tailored for Raspberry Pi, leveraging I2C communication, and includes advanced concurrency features, synchronization mechanisms, and comprehensive user-space tests.

## Table of Contents
- [Introduction to the Sensor](#introduction-to-the-sensor)
- [Features](#features)
- [Code Structure](#code-structure)
- [Build](#build)
- [Installation](#installation)
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

The sensor communicates via I2C (default address 0x68) and is widely used in robotics, VR/AR, and stabilization systems. This driver exposes its functionality through `/dev/mpu9250`, sysfs attributes, and the Linux input subsystem.

## Features

- **Kernel Integration**: I2C-based probe/remove, power management (suspend/resume), IRQ handling.
- **Data Access**: Read accelerometer, gyroscope, magnetometer, and quaternion data via ioctl, read/write registers, FIFO/DMP support, and calibration.
- **Concurrency**: Thread pools for parallel sensor reads, dynamic thread count, pause/resume threads.
- **Synchronization**: Recursive mutexes, read-write locks, condition variables, semaphores, barriers, notifiers for Inter-Thread Communication (ITC), and deadlock detection/prevention.
- **User-Space Tests**: Multi-threaded tests for sensor data (accel, gyro, mag, DMP) and IPC mechanisms (message queues, shared memory, FIFO, pipes) with signals, detached/joinable threads, and publisher-subscriber model.
- **Device Tree Overlay**: Configurable via Device Tree Source (DTS) for I2C, interrupts, and threading parameters (e.g., num_threads, semaphore_type).
- **Build and Documentation**: Makefile for building module, DTB, and tests, with Doxygen support and valgrind/helgrind for concurrency validation.

## Code Structure

The project consists of 8 files: 4 kernel driver files, 1 header, 2 user-space test programs, 1 Device Tree overlay, and 1 Makefile. Below is a detailed breakdown of each file, including the key knowledge and concepts applied.

### mpu9250.h
- **Purpose**: Defines constants, structs, enums, IOCTL commands, and function prototypes for the driver.
- **Key Features**:
  - Structs: `mpu9250_data` (with thread_pool, sync primitives like rwlock_t, semaphore), `mpu9250_mq_data` (for accel/gyro/mag/quat).
  - Enums: Scales (e.g., `ACCEL_SCALE_2G`), interrupts (e.g., `MPU9250_INTERRUPT_DATA_READY`).
  - IOCTLs: For reading sensors, setting scales, calibration, thread control.
- **Knowledge Used**:
  - Linux kernel data structures (struct, atomic_t).
  - Synchronization: mutex, rwlock, semaphore, notifier_head for ITC.
  - Thread management: kthread structs.
  - Atomic operations for deadlock detection.
  - Ensures multi-instance support and concurrency safety.

### mpu9250_ops.c
- **Purpose**: Implements core sensor operations: I2C read/write, initialization, FIFO/DMP setup, scale/sample rate configuration, data conversion, and calibration.
- **Key Features**:
  - I2C communication using regmap for registers (e.g., WHO_AM_I, PWR_MGMT_1).
  - FIFO/DMP initialization for efficient data handling.
  - Calibration with cleanup handlers for deferred cancellation.
- **Knowledge Used**:
  - Regmap API for I2C access.
  - Kthread cancellation (deferred with cleanup handlers).
  - Monitors (wait_event for resource availability).
  - Deadlock checks (atomic counters, try_lock).
  - Parallelism in calibration loops with interruptible waits.
  - Ensures low-latency, real-time data processing.

### mpu9250_driver.c
- **Purpose**: Core driver logic for probe/remove, sysfs attributes, thread pool management, power management, and input device registration.
- **Key Features**:
  - Probes I2C device with Device Tree matching.
  - Sysfs attrs for data/scale read/write.
  - Thread pool for parallel reads, pause/resume, ITC via notifiers.
- **Knowledge Used**:
  - I2C driver framework (probe/remove, DT match).
  - Kthread creation/binding for multi-core parallelism.
  - Barriers/wait-queues for thread synchronization.
  - Recursive mutexes, notifiers for ITC, atomic ops for deadlock prevention.
  - Input subsystem for event reporting.
  - Scalable for multi-core systems.

### mpu9250_fileops.c
- **Purpose**: Implements file operations for `/dev/mpu9250`: open, release, read, write, ioctl, poll, mmap, and IRQ/workqueue handling.
- **Key Features**:
  - Char device interface for user-space access.
  - Ioctl commands for sensor reads, calibration, thread control.
  - Poll and mmap for asynchronous access and shared buffers.
- **Knowledge Used**:
  - Char device (cdev, class_create, device_create).
  - Poll_table for non-blocking I/O.
  - Remap_vmalloc_range for shared memory.
  - Completion structs for event pairs.
  - Spinlocks/rwlocks for safe data access.
  - Workqueues for IRQ-driven reads.
  - Provides robust user-space interface with concurrency safety.

### mpu9250_ipc_test.c
- **Purpose**: User-space test program for IPC mechanisms (message queues, shared memory, FIFO, pipes) with multi-threading and synchronization.
- **Key Features**:
  - Tests IPC with pthread-based thread pools.
  - Supports signals, deferred cancellation, cleanup handlers.
  - Implements publisher-subscriber via callbacks, barriers for sync.
- **Knowledge Used**:
  - Pthread APIs (create, join, detach, cancel).
  - Semaphores for alternation/bounded waiting.
  - Condition variables for producer-consumer.
  - Notifiers/callbacks for publisher-subscriber.
  - Map-reduce pattern (implicit via parallel data processing).
  - Dining philosophers (semaphores for resource sync).
  - Demonstrates scalable IPC testing.

### user_space_test.c
- **Purpose**: User-space test program for sensor data (accel, gyro, mag, DMP) with multi-threading and kernel interaction.
- **Key Features**:
  - Tests ioctl calls for sensor reads, scales, and thread control.
  - Supports fork/exec, signals, and detached/joinable threads.
  - Implements assembly line pipeline (read-convert-sync-output).
- **Knowledge Used**:
  - Similar to ipc_test but focused on kernel-user interaction.
  - Pthread synchronization (barriers, monitors, rwlocks, recursive mutexes).
  - Dining philosophers (semaphores for resource sync).
  - Getopt for CLI parsing, mqueue for data transfer.
  - Tests robust kernel-user communication.

### mpu9250.dts
- **Purpose**: Device Tree overlay for Raspberry Pi, configuring I2C1, GPIO interrupts, and power supplies.
- **Key Features**:
  - Enables I2C1 with MPU9250 at address 0x68, AK8963 at 0x0C.
  - Configures interrupts (GPIO 17) and pinctrl.
  - Dynamic overrides for I2C speed, scales, threading params (num_threads, semaphore_type).
- **Knowledge Used**:
  - Device Tree syntax (overlays, fragments, compatible strings).
  - Pinctrl for GPIO configuration.
  - Dynamic parameterization for flexible hardware setup.
  - Allows hardware configuration without kernel recompilation.

### Makefile
- **Purpose**: Build script for kernel module, user-space tests, DTB, and documentation.
- **Key Features**:
  - Targets for kernel_module, user_progs, dtb, install, uninstall, clean, docs, thread_test.
  - Supports cross-compilation and dependency checks.
  - Valgrind/helgrind for concurrency testing.
- **Knowledge Used**:
  - Linux kernel build system (obj-m, modules_install).
  - Cross-compilation for ARM64 (Raspberry Pi).
  - Dependency checks for pthread and kernel threading.
  - Doxygen for documentation.
  - Ensures reproducible builds and concurrency validation.

## Build

To build all artifacts (kernel module, user-space tests, device tree overlay), run:

```bash
make all
```

To build specific components:
- Kernel module: `make kernel_module`
- Device tree overlay: `make dtb`
- User-space tests: `make user_progs`

To run concurrency tests with valgrind/helgrind:

```bash
make thread_test
```

To generate Doxygen documentation:

```bash
make docs
```

To clean build artifacts (keep sources):

```bash
make clean
```

To clean everything except sources:

```bash
make cleanall
```

Demo test files: `user_space_test.c` or `mpu9250_ipc_test.c`.

## Installation

To install the MPU9250 driver on a Raspberry Pi, follow these steps. Ensure kernel headers are installed (`sudo apt install raspberrypi-kernel-headers`).

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
Open the .dts file (e.g., with `nano bcm2711-rpi-4-b.dts`). Locate the I2C-1 section (under `&i2c1`). Add or modify to include the MPU9250 configuration from `mpu9250.dts`:
- Copy the `fragment@0` (I2C1 with mpu9250@68, AK8963 magnetometer).
- Copy `fragment@1` (GPIO interrupt pin).
- Copy `fragment@2` (pinctrl configuration).
- Ensure `status = "okay";` for I2C1.

### Step 4: Recompile and Reboot
Recompile the .dts file back to .dtb and reboot:

```bash
dtc -I dts -O dtb -o bcm2711-rpi-4-b.dtb bcm2711-rpi-4-b.dts
sudo reboot
```

**Note**: Replace filenames for your Raspberry Pi model.

### Step 5: Build and Install the Driver
1. Ensure all source files (`mpu9250_driver.c`, `mpu9250_fileops.c`, `mpu9250_ops.c`, `mpu9250.h`, `Makefile`, `mpu9250.dts`) are in the same directory.
2. Build the driver and DTB:
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

The driver creates `/dev/mpu9250` for user-space access. Use the following steps to interact with the sensor:

1. **Open Device**:
   ```c
   int fd = open("/dev/mpu9250", O_RDWR);
   if (fd < 0) perror("Failed to open /dev/mpu9250");
   ```

2. **Configure Scales and Sample Rate**:
   ```c
   ioctl(fd, MPU9250_IOCTL_SET_ACCEL_SCALE, ACCEL_SCALE_2G);
   ioctl(fd, MPU9250_IOCTL_SET_GYRO_SCALE, GYRO_SCALE_250DPS);
   ioctl(fd, MPU9250_IOCTL_SET_SAMPLE_RATE, 100);
   ```

3. **Read Sensor Data**:
   ```c
   struct mpu9250_sensor_data data;
   ioctl(fd, MPU9250_IOCTL_READ_ACCEL, &data);
   printf("Accel: %.2f %.2f %.2f g\n", data.values[0], data.values[1], data.values[2]);
   ```
   Similar ioctls for gyro (`MPU9250_IOCTL_READ_GYRO`), magnetometer (`MPU9250_IOCTL_READ_MAG`), and DMP quaternions (`MPU9250_IOCTL_READ_DMP`).

4. **Thread Control**:
   Pause/resume threads or set thread count:
   ```c
   ioctl(fd, MPU9250_IOCTL_PAUSE_THREADS);
   ioctl(fd, MPU9250_IOCTL_SET_NUM_THREADS, 4);
   ```

5. **Run Tests**:
   Compile and run user-space tests:
   ```bash
   ./user_space_test --accel --gyro --mag --dmp --sample-rate 100
   ./mpu9250_ipc_test --mq --shm --fifo --pipe
   ```

## Expected Results

When reading sensor values (e.g., via ioctl or test programs) with the MPU9250 at rest on a flat surface:
- **Accelerometer**: ~[0.00, 0.00, 1.00] g (gravity on Z-axis, ±0.05g noise after calibration).
- **Gyroscope**: ~[0.00, 0.00, 0.00] °/s (no rotation, ±0.1°/s noise).
- **Magnetometer**: Varies by location, e.g., [X, Y, Z] in μT (Earth’s magnetic field ~25-65 μT, depends on calibration and environment).
- **Quaternions (DMP)**: ~[1.00, 0.00, 0.00, 0.00] for identity orientation (no rotation).

Example output from `user_space_test` or callbacks:
```
Callback: Accel: 0.02 0.01 0.98, Gyro: 0.05 -0.03 0.02, Mag: 30.50 42.10 -48.20, Quat: 1.00 0.00 0.00 0.00
```

**Notes**:
- Calibration (via `MPU9250_IOCTL_CALIBRATE`) adjusts offsets to minimize bias.
- Errors like `-EIO` indicate I2C communication issues (check wiring or I2C address).
- Data accuracy depends on sensor placement, calibration, and environmental factors (e.g., magnetic interference).

## License

This project is licensed under the **GPL** (see `mpu9250_driver.c` for details). Author: Nguyen Nhan. Version: 2.3.