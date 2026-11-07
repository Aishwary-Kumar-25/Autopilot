#include "hal/posix/posix_hal.hpp"
#include "vehicles/uav/uav_vehicle.hpp"
#include "vehicles/ugv/ugv_vehicle.hpp"
#include "vehicles/rov/rov_vehicle.hpp"
#include "system/scheduler.hpp"
#include "system/telemetry.hpp"
#include <iostream>

using namespace autopilot;

int main(int argc, char** argv) {
    std::cout << "========================================================\n";
    std::cout << "        E-CLUB MODULAR PLUG-AND-PLAY AUTOPILOT         \n";
    std::cout << "       Target: STM32 Class / POSIX Simulation         \n";
    std::cout << "========================================================\n";

    // Select vehicle type via command argument (default: UAV)
    std::string vtype = "uav";
    if (argc > 1) {
        vtype = argv[1];
    }

    hal::PosixHAL hal;

    // Attach simulated sensors to I2C bus for demonstration
    hal::PosixI2CDevice::RegisterMap mpu;
    mpu[0x75] = 0x68; // WHO_AM_I MPU6050
    mpu[0x6B] = 0x00;
    mpu[0x3F] = 0x10; // Accel Z 1g
    hal.i2c().attach_device(0x68, mpu);

    hal::PosixI2CDevice::RegisterMap bmp;
    bmp[0xD0] = 0x58; // BMP280
    // Nominal room temperature and sea level pressure registers
    bmp[0xF7] = 0x65;
    bmp[0xF8] = 0x5A;
    bmp[0xF9] = 0xC0;
    bmp[0xFA] = 0x7E;
    bmp[0xFB] = 0xE9;
    bmp[0xFC] = 0x00;
    hal.i2c().attach_device(0x76, bmp);

    std::unique_ptr<vehicles::VehicleBase> vehicle;
    if (vtype == "ugv" || vtype == "rover") {
        vehicle = std::make_unique<vehicles::UGVVehicle>(&hal);
    } else if (vtype == "rov" || vtype == "sub") {
        vehicle = std::make_unique<vehicles::ROVVehicle>(&hal);
    } else {
        vehicle = std::make_unique<vehicles::UAVVehicle>(&hal);
    }

    if (!vehicle->init()) {
        std::cerr << "Failed to initialize vehicle!\n";
        return 1;
    }

    std::cout << "Loaded Vehicle: " << vehicle->get_type_name() << "\n";

    std::string err;
    if (!vehicle->arm(err)) {
        std::cerr << "Pre-arm check failed: " << err << "\n";
    } else {
        std::cout << "Vehicle successfully armed!\n";
    }

    system::Scheduler scheduler(hal.get_time());
    system::Telemetry telem(hal.get_uart(), vehicle.get());

    // 100 Hz Fast Flight Loop
    scheduler.add_task("flight_loop", [&]() {
        vehicle->step(0.01f);
    }, 100);

    // 10 Hz Telemetry Attitude Stream
    scheduler.add_task("telem_attitude", [&]() {
        telem.send_attitude();
    }, 10);

    // 1 Hz Heartbeat & Sensor Health
    scheduler.add_task("telem_heartbeat", [&]() {
        telem.send_heartbeat();
        telem.send_sensor_status();
    }, 1);

    std::cout << "Starting real-time flight loop simulation (100 steps)...\n";
    for (int i = 0; i < 100; ++i) {
        scheduler.run();
        hal.time().delay_ms(10);
    }

    std::cout << "\n[Telemetry Output Stream]:\n";
    std::cout << hal.uart().get_output_history() << "\n";

    std::cout << "Simulation cycle finished successfully.\n";
    return 0;
}
