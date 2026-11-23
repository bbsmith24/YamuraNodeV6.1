# PlatformIO post-build script: write firmware.hex next to firmware.bin,
# for flashing with the vendor's mcuisp tool over the board's USB-C (CH340) port.
Import("env")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(
        "$OBJCOPY -O ihex $BUILD_DIR/${PROGNAME}.elf $BUILD_DIR/${PROGNAME}.hex",
        "Building $BUILD_DIR/${PROGNAME}.hex",
    ),
)
