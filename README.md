A simple bit-banging adapter for dap-rs
=======================================

This crate makes it very easy to implement a CMSIS-DAP probe in Rust. All you need is a transport layer and two trait implementations.

For an example, have a look at [fruitfly](https://github.com/bugadani/fruitfly), an alternative RPi debugprobe firmware.
