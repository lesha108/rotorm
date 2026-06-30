cargo build --release
cargo objcopy --bin rotorm --target thumbv7em-none-eabihf --release -- -O binary rotorm.bin
cargo embed --release --chip STM32F411CEUx

