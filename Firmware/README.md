
# 📦 Arduino MCP2515 CAN Framework

This project provides a reusable framework for CAN communication between Arduino devices using the **MCP2515** module. It supports both sending and receiving messages, including arbitrarily long strings via a simple custom protocol.

---

## 🛠 Requirements

- Ubuntu system (22.04+ recommended)
- Arduino board (e.g. Uno)
- MCP2515 CAN controller module
- A USB connection to your Arduino (`/dev/ttyUSB0` or similar)

---

## 🚀 Getting Started

1. **Clone or download** the project folder.
2. Open a terminal in the project directory.

---

## 📍 Step 1: Install Dependencies

Run the following script to install the Arduino CLI and required libraries:

```bash
./init.sh
```

This will:
- Install the [Arduino CLI](https://arduino.github.io/arduino-cli/)
- Install the AVR core
- Install the MCP_CAN library

---

## 🧱 Step 2: Build & Upload

Use the `build.sh` script to compile and upload the appropriate sketch to your Arduino:

```bash
./build.sh ClientA
./build.sh ClientB

```

> If no argument is passed, the script defaults to `sender`.

Make sure to edit the `PORT` variable in `build.sh` to match your Arduino's port (e.g., `/dev/ttyUSB0`, `/dev/ttyACM0`, etc).

---

## 📂 Project Structure

```
.
├── init.sh                 # Installs Arduino CLI and libraries
├── build.sh                # Compiles and uploads sender or receiver
├── MessageCodes.h          # Enum definitions for message types
├── CANBusManager.h / .cpp  # Core reusable CAN communication code
├── ClientA.ino         # Example: sends CAN messages
├── ClientB.ino       # Example: receives CAN messages
├── README.md               # You're here!
```

---

## ✅ Example Output

When you run:

```bash
./build.sh ClientA
```

You should see:

```
Compiling main_sender.ino...
Uploading to /dev/ttyUSB0...
Done.
```

