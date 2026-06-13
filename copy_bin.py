# copy_bin.py - Automatycznie kopiuje firmware.bin po kompilacji
Import("env")

import os
import shutil

def copy_bin_callback(source, target, env):
    build_dir = env['BUILD_DIR']
    project_dir = env['PROJECT_DIR']
    firmware_bin = os.path.join(build_dir, 'firmware.bin')
    dest_path = os.path.join(project_dir, 'firmware.bin')
    
    if os.path.exists(firmware_bin):
        shutil.copy2(firmware_bin, dest_path)
        print(f"Firmware copied to: {dest_path}")

# Add post-build action
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_bin_callback)