"""
Example BSC20X_pythonnet.py
Example Date of Creation: 2024-04-19
Example Date of Last Modification on Github: 2024-04-19
Version of Python used for Testing: 3.9
==================
Example Description: This example controls the BSC200 series (Using the HDR50/M stage)
"""
import os
import time
import sys
import clr

clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.DeviceManagerCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\Thorlabs.MotionControl.GenericMotorCLI.dll")
clr.AddReference("C:\\Program Files\\Thorlabs\\Kinesis\\ThorLabs.MotionControl.Benchtop.StepperMotorCLI.dll")
from Thorlabs.MotionControl.DeviceManagerCLI import *
from Thorlabs.MotionControl.GenericMotorCLI import *
from Thorlabs.MotionControl.Benchtop.StepperMotorCLI import *
from System import Decimal  # necessary for real world units

def main2():
    
    DeviceManagerCLI.BuildDeviceList()

    # create new device
    serial_no = "70470634"  # Replace this line with your device's serial number

    # Connect, begin polling, and enable
    device = BenchtopStepperMotor.CreateBenchtopStepperMotor(serial_no)
    device.Connect(serial_no)
    time.sleep(0.25)  # wait statements are important to allow settings to be sent to the device

    # For benchtop devices, get the channel
    channel = device.GetChannel(2)

    print(f"{help(channel.MoveAbsolute)}")

def main():

    DeviceManagerCLI.BuildDeviceList()

    # create new device
    serial_no = "70470634"  # Replace this line with your device's serial number

    # Connect, begin polling, and enable
    device = BenchtopStepperMotor.CreateBenchtopStepperMotor(serial_no)
    device.Connect(serial_no)
    time.sleep(0.25)  # wait statements are important to allow settings to be sent to the device

    # For benchtop devices, get the channel
    channel = device.GetChannel(2)
    
    # Ensure that the device settings have been initialized
    if not channel.IsSettingsInitialized():
        channel.WaitForSettingsInitialized(10000)  # 10 second timeout
        assert channel.IsSettingsInitialized() is True

    # Start polling and enable
    channel.StartPolling(250)  # 250ms polling rate
    time.sleep(0.5)
    channel.EnableDevice()
    time.sleep(0.25)  # Wait for device to enable

    # Load any configuration settings needed by the controller/stage
    channel_config = channel.LoadMotorConfiguration(channel.DeviceID)
    chan_settings = channel.MotorDeviceSettings

    channel.GetSettings(chan_settings)

    channel_config.DeviceSettingsName = 'NanoMax300'

    channel_config.UpdateCurrentConfiguration()

    channel.SetSettings(chan_settings, True, False)

    # Get parameters related to homing/zeroing/other

    # Home or Zero the device (if a motor/piezo)
    print("Homing Motor")
    channel.Home(6000)
    print("Homing Completed")

    time.sleep(2)
    print("Moving...")
    channel.MoveTo(Decimal(3), 6000)
    # channel.MoveAbsolute(Decimal(7), 6000)
    print(f"Position = {channel.DevicePosition}")
    time.sleep(2)

    # Stop Polling and Disconnect
    channel.StopPolling()
    device.Disconnect()

if __name__ == "__main__":
    main()
