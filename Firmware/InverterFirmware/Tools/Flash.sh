#!/usr/bin/env python3
import os
import time
import subprocess
import shutil
import sys
import re
from pathlib import Path

def clean_device_name(device):
    """Remove special characters from device names."""
    return re.sub(r'[^a-zA-Z0-9\/_-]', '', device)

def run_command(cmd, sudo=False):
    """Run a command with optional sudo."""
    if sudo:
        cmd = ['sudo'] + cmd
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        print(f"Command failed: {' '.join(cmd)}")
        if e.stderr.strip():
            print(f"Error: {e.stderr.strip()}")
        return None

def find_pico_partition():
    """Find the Raspberry Pi Pico partition in bootloader mode."""
    # Try lsblk with raw output
    result = run_command(['lsblk', '-o', 'NAME,LABEL,FSTYPE,MOUNTPOINT', '-n'])
    if result:
        for line in result.splitlines():
            if "RPI-RP2" in line or ("FAT" in line and "RP2" in line):
                parts = line.split()
                if len(parts) >= 3 and "vfat" in line:
                    # Clean the device name and find the actual partition
                    base_device = clean_device_name(parts[0])
                    if base_device[-1].isdigit():
                        return f"/dev/{base_device}"
                    else:
                        # Check for partitions (sdb1, sdb2, etc.)
                        for part in range(1, 5):
                            test_device = f"/dev/{base_device}{part}"
                            if os.path.exists(test_device):
                                return test_device
    
    # Alternative method using blkid
    result = run_command(['sudo', 'blkid'])
    if result:
        for line in result.splitlines():
            if "RPI-RP2" in line or "LABEL_FATBOOT=\"RPI-RP2\"" in line:
                device = line.split(':')[0]
                return clean_device_name(device)
    
    return None

def mount_pico(partition):
    """Mount the Pico partition using sudo."""
    mount_dir = f"/media/{os.getlogin()}/RPI-RP2"
    
    # Create mount directory if needed
    run_command(['mkdir', '-p', mount_dir], sudo=True)
    run_command(['chown', f'{os.getlogin()}:{os.getlogin()}', mount_dir], sudo=True)
    
    # Mount with explicit filesystem type and options
    if run_command(['mount', '-t', 'vfat', '-o', 'uid=1000,gid=1000', partition, mount_dir], sudo=True):
        return mount_dir
    
    return None

def unmount_pico(partition_or_mount):
    """Unmount the Pico."""
    return run_command(['umount', partition_or_mount], sudo=True)

def wait_for_pico_disconnect(pico_device, timeout=10):
    """Wait for the Pico to disconnect after flashing."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        current_device = find_pico_partition()
        if current_device != pico_device:
            return True
        time.sleep(0.5)
    return False

def main():
    print("Raspberry Pi Pico Auto-Flash Tool")
    print("---------------------------------")
    
    # Define the path to the UF2 file
    script_dir = Path(__file__).parent.resolve()
    uf2_file = script_dir.parent / "Build" / "InverterFirmware.uf2"
    
    if not uf2_file.exists():
        print(f"\nError: UF2 file not found at {uf2_file}")
        print("Please build the firmware first.")
        sys.exit(1)

    # Wait for Pico to be connected
    print("\n1. Waiting for Raspberry Pi Pico in bootloader mode...")
    print("   (Hold BOOTSEL button while connecting USB)")
    
    pico_partition = None
    for _ in range(30):  # 30 second timeout
        pico_partition = find_pico_partition()
        if pico_partition:
            break
        time.sleep(1)
    
    if not pico_partition:
        print("\nError: Pico not detected. Please:")
        print("- Check USB connection")
        print("- Make sure Pico is in bootloader mode (RPI-RP2)")
        sys.exit(1)
    
    print(f"\n2. Pico partition found at {pico_partition}")
    
    # Mount the Pico
    print("\n3. Mounting Pico...")
    mount_point = mount_pico(pico_partition)
    
    if not mount_point:
        print("\nError: Could not mount Pico partition")
        print("Attempting direct UF2 copy...")
        
        # Try direct copy to the partition
        try:
            print(f"Copying {uf2_file} to {pico_partition}")
            run_command(['sudo', 'cp', str(uf2_file), pico_partition], sudo=True)
            print("Flash successful using direct copy method!")
            print("\nPlease manually reset your Pico")
            sys.exit(0)
        except Exception as e:
            print(f"Direct copy failed: {e}")
            print("\nFinal recovery options:")
            print("1. Try mounting manually:")
            print(f"   sudo mkdir -p /media/{os.getlogin()}/RPI-RP2")
            print(f"   sudo mount -t vfat -o uid=1000,gid=1000 {pico_partition} /media/{os.getlogin()}/RPI-RP2")
            print("2. Or copy the UF2 file manually:")
            print(f"   sudo cp {uf2_file} {pico_partition}")
            sys.exit(1)
    
    print(f"   Pico mounted at {mount_point}")
    
    # Flash the UF2 file
    print("\n4. Flashing firmware...")
    try:
        dest_path = Path(mount_point) / uf2_file.name
        print(f"   Copying {uf2_file} to {dest_path}")
        shutil.copy(uf2_file, dest_path)
        print("   Flash successful!")
    except Exception as e:
        print(f"   Flash failed: {e}")
        sys.exit(1)
    
    # Unmount the Pico
    print("\n5. Unmounting Pico...")
    if unmount_pico(mount_point):
        print("   Unmount successful")
    else:
        print("   Warning: Could not unmount Pico (it may have auto-unmounted)")
    
    # Wait for Pico to disconnect (indicating reboot)
    print("\n6. Waiting for Pico to reboot...")
    if wait_for_pico_disconnect(pico_partition):
        print("   Pico disconnected and rebooted successfully!")
    else:
        print("   Warning: Pico did not disconnect - it may not have rebooted")

if __name__ == "__main__":
    main()