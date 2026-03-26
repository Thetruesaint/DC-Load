import os
import shutil

Import("env")

env_name = env["PIOENV"]

if env_name == "sim":
    output_filename = "DCLoad_main_sim.elf"
    firmware_path = os.path.join(env.subst("$BUILD_DIR"), "firmware.elf")
    output_path = os.path.join(env.subst("$BUILD_DIR"), output_filename)

    def rename_firmware(source, target, env):
        if os.path.exists(firmware_path):
            shutil.copy(firmware_path, output_path)

        wokwi_toml_path = os.path.join(env.subst("$PROJECT_DIR"), "wokwi.toml")
        with open(wokwi_toml_path, "w", encoding="utf-8") as wokwi_toml:
            wokwi_toml.write(
                f"""[wokwi]
version = 1
firmware = ".pio/build/sim/{output_filename}"
elf = ".pio/build/sim/{output_filename}"
"""
            )
            print(f"Updated Wokwi firmware path: {output_filename}")

    env.AddPostAction("$BUILD_DIR/firmware.elf", rename_firmware)
