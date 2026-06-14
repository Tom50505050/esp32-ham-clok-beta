import os
import shutil
from SCons.Script import Import

Import("env")

def copy_firmware(source, target, env):
    # Ścieżka źródłowa to skompilowany firmware.bin wewnątrz .pio/build/
    source_path = str(source[0])
    # Ścieżka docelowa to główny folder projektu (rodzic ESP32-HAM-CLOCK-MERGED)
    project_dir = env.subst("$PROJECT_DIR")
    parent_dir = os.path.dirname(project_dir)
    target_path = os.path.join(parent_dir, "firmware.bin")
    
    print(f"--- [CUSTOM SCRIPT] Kopiowanie {source_path} do {target_path} ---")
    shutil.copyfile(source_path, target_path)

# Powiązanie funkcji z akcją po wygenerowaniu pliku binarnego
env.AddPostAction("buildprog", copy_firmware)
