# MPU9250 Kernel Driver for Raspberry Pi (RUN follow the file mpu9250-kernel-module-thread-advance-expert)

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
- **FIFO Buffer**: Hardware buffer (up to 4096 bytes) for efficient data collection.
- **Interrupts**: Supports data-ready, FIFO overflow, and DMP interrupts via GPIO 17.

The sensor communicates via I2C (default address 0x68 for MPU9250, 0x0C for magnetometer) and is widely used in robotics, VR/AR, and stabilization systems. This driver exposes functionality through `/dev/mpu9250`, sysfs attributes (`/sys/class/mpu9250_class/mpu9250/mpu9250_data`), and the Linux input subsystem.

## Features

- **Kernel Integration**: I2C-based probe/remove, power management via runtime PM, IRQ handling with atomic operations, and race condition prevention using custom read-write locks and semaphores.
- **Data Access**: Read accelerometer, gyroscope, magnetometer, and DMP quaternion data via file operations (read/write/ioctl/poll/mmap) with blocking/non-blocking support.
- **Concurrency**: Thread pools for parallel sensor reads (up to 8 threads), dynamic thread count configuration, pause/resume functionality, using POSIX threads in user space (`user_space_test.c`, `map_reduce_test.c`) and kernel threads (`mpu9250_driver.c`).
- **Synchronization**: Custom recursive mutexes, read-write locks (`custom_rwlock.c`), weak semaphores (`weak_semaphore.c`), condition variables, barriers, notifiers for Inter-Thread Communication (ITC), and dining philosophers pattern to prevent deadlocks.
- **User-Space Tests**: Multi-threaded test programs (`user_space_test.c`, `mpu9250_ipc_test.c`, `map_reduce_test.c`, `bridge_problem_test.c`, `assembly_line_test.c`, `vaccination_drive_test.c`) demonstrating sensor data processing, IPC (message queues, shared memory, FIFO, pipes), and concurrency patterns (map-reduce, pipeline, bounded resources).
- **Calibration**: Supports sensor calibration via `MPU9250_IOCTL_CALIBRATE` with dining philosophers synchronization.
- **Device Tree**: Configurable via `mpu9250.dts` with overrides for I2C address, interrupt pin, scales, and thread settings.
- **Memory Management**: Supports mmap for FIFO buffer access and dynamic buffer allocation (`mpu9250_alloc_buffers`).
- **Signal Handling**: Handles SIGINT, SIGTERM, SIGUSR1 in user-space tests for graceful termination.
- **Debugging**: Sysfs attributes for data inspection, Valgrind/Helgrind support for concurrency testing, and deadlock detection (`mpu9250_check_deadlock`).

## Code Structure

The project includes kernel module source files and user-space test programs:

- **Kernel Module**:
  - `mpu9250_driver.c`: Main driver, handles I2C probe/remove, input device registration.
  - `mpu9250_fileops.c`: Implements file operations (open, read, write, ioctl, poll, mmap).
  - `mpu9250_ops.c`: Sensor read/write, initialization, calibration, and scale configuration.
  - `mpu9250_timer.c`: Periodic timer for sensor reads.
  - `custom_rwlock.c`: Custom read-write lock for kernel synchronization.
  - `weak_semaphore.c`: Custom semaphore for resource access control.
  - `mpu9250.dts`: Device tree overlay for I2C and GPIO configuration.

- **User-Space Tests**:
  - `user_space_test.c`: Tests basic sensor reads with thread pools and synchronization.
  - `mpu9250_ipc_test.c`: Tests IPC mechanisms (message queues, shared memory, FIFO, pipes).
  - `map_reduce_test.c`: Implements map-reduce pattern for sensor data aggregation.
  - `bridge_problem_test.c`: Simulates bounded resource access (cars on a bridge).
  - `assembly_line_test.c`: Simulates a pipeline for sensor data processing.
  - `vaccination_drive_test.c`: Simulates bounded resource allocation (vaccination stations).

- **Supporting Files**:
  - `mpu9250.h`: Header with constants, structs, and function prototypes.
  - `Makefile`: Builds kernel module and user-space programs, supports Valgrind/Helgrind.
  - `README.md`: This file.

## Build

To build all artifacts (kernel module, user programs, and device tree overlay) use:

```bash
make all
```

To build only specific components, use one of the following:

```bash
make kernel_module
make dtb
make user_progs
```

To generate documentation (optional):

```bash
make docs
```

To check dependencies:

```bash
make check_deps
```

To delete all build artifacts:

```bash
make clean
```

To delete all files except sources:

```bash
make cleanall  # Note: Add this target to Makefile if needed
```

**Note**: Ensure kernel headers are installed (`/lib/modules/$(uname -r)/build`) and tools like `gcc`, `make`, `dtc`, `valgrind`, `doxygen` are available. No additional libraries beyond standard ones are required.

## Install

To install the MPU9250 driver for Raspberry Pi, follow these steps:

**Step 1**: Navigate to the `/boot` directory of the Raspberry Pi (optional if using the provided device tree overlay):

```bash
cd /boot
```

**Step 2**: If integrating the MPU9250 into the base device tree (instead of using the provided `mpu9250.dts` overlay), convert the Raspberry Pi device tree blob (.dtb) to a device tree source (.dts) file:

```bash
dtc -I dtb -O dts -o bcm2710-rpi-3-b.dts bcm2710-rpi-3-b.dtb
```

**Note**: Replace `bcm2710-rpi-3-b.dtb` with the appropriate .dtb file for your Raspberry Pi model (e.g., `bcm2711-rpi-4-b.dtb` for Pi 4). Ensure compatibility with your model.

**Step 3**: Open the generated .dts file (e.g., `bcm2710-rpi-3-b.dts`) and locate the I2C-1 section (e.g., `i2c1`). Add or modify the MPU9250 node to match the configuration in `mpu9250.dts`, for example:

```dts
mpu9250@68 {
    compatible = "invensense,mpu9250";
    reg = <0x68>;
    interrupt-parent = <&gpio>;
    interrupts = <17 0x2>;
    pinctrl-names = "default";
    pinctrl-0 = <&mpu9250_pins>;
    status = "okay";
};
```

Save the changes.

**Step 4**: Recompile the .dts file back to .dtb and reboot the Raspberry Pi:

```bash
dtc -I dts -O dtb -o bcm2710-rpi-3-b.dtb bcm2710-rpi-3-b.dts
sudo reboot
```

**Preferred Method (Using Device Tree Overlay)**: The project includes `mpu9250.dts` as a pre-configured overlay, making manual .dts editing unnecessary in most cases. Instead:

- Build the overlay:

```bash
make dtb
```

- Copy to overlays:

```bash
sudo cp mpu9250.dtb /boot/overlays/mpu9250.dtbo
```

- Apply overlay:

```bash
sudo dtoverlay mpu9250
```

- To remove overlay:

```bash
sudo dtoverlay -r mpu9250
```

**Step 5**: Build and Install the Driver

- Ensure the source files and `Makefile` are in the same folder. Build the kernel module:

```bash
make kernel_module
```

- This generates `mpu9250_driver.ko` (and related .ko files for `mpu9250_fileops.o`, etc.).

- Install the driver:

```bash
sudo make install
```

- This installs modules to `/lib/modules/$(uname -r)/extra`, copies `mpu9250.dtbo` to `/boot/overlays`, and runs `depmod -a`.

- Load the module:

```bash
sudo insmod mpu9250_driver.ko
```

- Alternatively, after `make install`, use:

```bash
sudo modprobe mpu9250_driver
```

- Check installation status:

```bash
dmesg | grep mpu9250
```

- To remove the module:

```bash
sudo rmmod mpu9250_driver
```

**Note**: Ensure I2C is enabled in `/boot/config.txt` with `dtparam=i2c_arm=on`. Connect the MPU9250 to GPIO 2/3 (SDA/SCL) and GPIO 17 (interrupt). The device appears as `/dev/mpu9250`.

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
   struct mpu9250_mq_data data;
   read(fd, &data, sizeof(data));
   printf("Accel: %.2f %.2f %.2f g\n", data.accel[0], data.accel[1], data.accel[2]);
   ```
   Alternatively, use ioctls:
   ```c
   struct mpu9250_sensor_data sensor_data;
   ioctl(fd, MPU9250_IOCTL_READ_ACCEL, &sensor_data);
   printf("Accel: %.2f %.2f %.2f g\n", sensor_data.values[0], sensor_data.values[1], sensor_data.values[2]);
   ```
   Similar ioctls exist for gyro (`MPU9250_IOCTL_READ_GYRO`), magnetometer (`MPU9250_IOCTL_READ_MAG`), and DMP quaternions (`MPU9250_IOCTL_READ_DMP`).

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
   ./map_reduce_test
   ./bridge_problem_test
   ./assembly_line_test
   ./vaccination_drive_test
   ```
   Tests demonstrate:
   - `user_space_test`: Multi-threaded sensor reads with synchronization (mutex, condition variables, rwlocks, dining philosophers).
   - `mpu9250_ipc_test`: IPC mechanisms (POSIX message queues, shared memory, FIFO, pipes).
   - `map_reduce_test`: Map-reduce pattern for sensor data aggregation.
   - `bridge_problem_test`: Bounded resource access (cars crossing a bridge).
   - `assembly_line_test`: Pipeline processing of sensor data.
   - `vaccination_drive_test`: Bounded resource allocation (vaccination stations).
   Tests use fork(), signals (SIGINT, SIGTERM, SIGUSR1), POSIX threads, and synchronization primitives.

6. **Memory Mapping** (System Call: mmap):
   ```c
   void *fifo_map = mmap(NULL, MPU9250_MAX_FIFO, PROT_READ, MAP_SHARED, fd, 0);
   if (fifo_map == MAP_FAILED) perror("mmap failed");
   munmap(fifo_map, MPU9250_MAX_FIFO);
   ```

7. **Concurrency Testing**:
   ```bash
   make thread_test
   ```
   Runs tests with Valgrind/Helgrind to detect races and deadlocks.

## Expected Results

With the MPU9250 at rest on a flat surface (after calibration via `MPU9250_IOCTL_CALIBRATE`):
- **Accelerometer**: ~[0.00, 0.00, 1.00] g (±0.05g noise).
- **Gyroscope**: ~[0.00, 0.00, 0.00] °/s (±0.1°/s noise).
- **Magnetometer**: Location-dependent, e.g., [X, Y, Z] in μT (~25-65 μT).
- **Quaternions (DMP)**: ~[1.00, 0.00, 0.00, 0.00] for no rotation.

Example output from tests (e.g., `map_reduce_test` or `user_space_test`):
```
Callback: Accel: 0.02 0.01 0.98, Gyro: 0.05 -0.03 0.02, Mag: 30.50 42.10 -48.20, Quat: 1.00 0.00 0.00 0.00
```

**Notes**:
- Calibration uses dining philosophers for resource synchronization to prevent deadlocks.
- Errors like `-EIO` indicate I2C communication issues (check wiring, I2C address 0x68, or device tree).
- Concurrency tests (`make thread_test`) confirm thread safety with Valgrind/Helgrind.
- Some tests (e.g., `map_reduce_test`, `assembly_line_test`) may show incomplete data due to threading issues (e.g., incorrect thread ID calculation); see code for fixes.

## License

Licensed under the **GPL** (see `mpu9250_driver.c`). Author: Nguyen Nhan. Version: 2.3.
