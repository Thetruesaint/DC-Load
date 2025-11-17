import os
Import("env")

env_name = env["PIOENV"]

if env_name == "sim":
    branch_name = "main"  # o usa git si querés
    output_filename = f"DCLoad_{branch_name}_sim.elf"
    firmware_path = os.path.join(env.subst("$BUILD_DIR"), "firmware.elf")
    new_firmware_path = os.path.join(env.subst("$BUILD_DIR"), output_filename)

    def rename_firmware(source, target, env):
        import shutil
        if os.path.exists(firmware_path):
            shutil.copy(firmware_path, new_firmware_path)
            print(f"✅ Copied firmware to {new_firmware_path}")
        else:
            print("⚠️ firmware.elf not found!")

        wokwi_toml_path = os.path.join(env.subst("$PROJECT_DIR"), "wokwi.toml")
        with open(wokwi_toml_path, "w") as wokwi_toml:
            wokwi_toml.write(f"""[wokwi]
version = 1
firmware = ".pio/build/sim/{output_filename}"
elf = ".pio/build/sim/{output_filename}"
""")
            print(f"✅ Updated Wokwi firmware path: {output_filename}")

    env.AddPostAction("$BUILD_DIR/firmware.elf", rename_firmware)

