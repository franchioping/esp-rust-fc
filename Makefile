
run-sim:
	CARGO_TARGET_DIR="target-sim" \
	cargo run -p flight-sim 

build-sim:
	CARGO_TARGET_DIR="target-sim" \
	cargo build -p flight-sim 


flash-fw:
	CARGO_TARGET_DIR="target-embed" \
	CARGO_BUILD_TARGET="xtensa-esp32s3-none-elf" \
	CARGO_TARGET_XTENSA_ESP32S3_NONE_ELF_RUNNER="espflash flash --monitor" \
	CARGO_BUILD_RUSTFLAGS="-C link-arg=-nostartfiles -Z stack-protector=all" \
	DEFMT_LOG="info" \
	cargo +esp run -p firmware -Z build-std=alloc,core


build-fw:
	CARGO_TARGET_DIR="target-embed" \
	CARGO_BUILD_TARGET="xtensa-esp32s3-none-elf" \
	CARGO_TARGET_XTENSA_ESP32S3_NONE_ELF_RUNNER="espflash flash --monitor" \
	CARGO_BUILD_RUSTFLAGS="-C link-arg=-nostartfiles -Z stack-protector=all" \
	DEFMT_LOG="info" \
	cargo +esp build -p firmware -Z build-std=alloc,core


