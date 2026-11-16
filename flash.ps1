git pull && cargo build --release --target thumbv7em-none-eabihf && probe-rs download --chip STM32F407IGTx target/thumbv7em-none-eabihf/release/xcore407i_blinker
