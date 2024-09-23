import usb.core
import usb.util

# Replace with your device's vendor ID and product ID
VID = 0xcafe
PID = 16384  # Example Product ID
VID_STMPROJECT = 1155
PID_STMPROJECT = 22336

# Find the USB device
dev = usb.core.find(idVendor=VID, idProduct=PID)

if dev is None:
    raise ValueError("Device not found")
    exit()

if dev.is_kernel_driver_active(0):
    try:
        dev.detach_kernel_driver(0)
    except usb.core.USBError as e:
        print(f"Could not detach kernel driver: {e}")
        exit()
# Set the active configuration
dev.set_configuration()

# Get the endpoint address for the bulk IN transfer
# Replace with your device's actual endpoint address (e.g., 0x81 for IN endpoint)
bulk_in_endpoint = 0x81

# Define the size of the data to read
data_length = 64  # Adjust this based on the expected data size

# Perform the bulk read
try:
    # Read data from the bulk IN endpoint
    data = dev.read(bulk_in_endpoint, data_length, timeout=5000)
    print(f"Received data: {data}")
except usb.core.USBError as e:
    print(f"Error reading from USB: {e}")


#https://chatgpt.com/c/9841408f-ba69-44aa-86a5-cd698bab91dd