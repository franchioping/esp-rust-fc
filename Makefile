
clean:
	rm -rf ./targe*


run-sim:
	CARGO_TARGET_DIR="target-sim" \
	cargo run -p flight-sim 

build-sim:
	CARGO_TARGET_DIR="target-sim" \
	cargo build -p flight-sim 


run-view:
	CARGO_TARGET_DIR="target-view" \
	WINIT_UNIX_BACKEND=x11 \
	cargo run -p telemetry-viewer 

build-view:
	CARGO_TARGET_DIR="target-view" \
	cargo build -p telemetry-viewer 


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


