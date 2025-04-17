A simple bit-banging adapter for dap-rs
=======================================

This crate makes it very easy to implement a CMSIS-DAP probe in Rust. All you need is a transport layer and two trait implementations.

Example implementers:
- [fruitfly](https://github.com/bugadani/fruitfly), an alternative RPi debugprobe firmware.
- [espdap](https://github.com/bugadani/espdap), an ESP32-S2/S3 demo.
