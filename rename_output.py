import os
Import("env")

env_name = env["PIOENV"]

if env_name == "sim":
    # Obtener la rama actual desde Git
    branch_name = "unknown"
    try:
        branch_name = os.popen("git rev-parse --abbrev-ref HEAD").read().strip()
    except:
        pass

    # Definir el nombre del firmware basado en la rama
    output_filename = f"firmware_{branch_name}.hex"
    firmware_path = os.path.join(env.subst("$BUILD_DIR"), "firmware.hex")
    new_firmware_path = os.path.join(env.subst("$BUILD_DIR"), output_filename)

    # Renombrar el firmware después de la compilación
    def rename_firmware(source, target, env):
        if os.path.exists(new_firmware_path):
            os.remove(new_firmware_path)  # 🔹 Eliminar el archivo si ya existe

        if os.path.exists(firmware_path):
            os.rename(firmware_path, new_firmware_path)
            print(f"✅ Renamed firmware to {new_firmware_path}")

        # Modificar wokwi.toml para apuntar al nuevo firmware
        wokwi_toml_path = os.path.join(env.subst("$PROJECT_DIR"), "wokwi.toml")
        with open(wokwi_toml_path, "w") as wokwi_toml:
            wokwi_toml.write(f"""[wokwi]
version = 1
firmware = ".pio/build/sim/{output_filename}"
elf = ".pio/build/sim/firmware.elf"
""")
            print(f"✅ Updated Wokwi firmware path in wokwi.toml: {output_filename}")

    env.AddPostAction("$BUILD_DIR/firmware.hex", rename_firmware)