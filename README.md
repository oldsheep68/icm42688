# icm42688

![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/jessebraham/icm42688/ci.yaml?label=CI&logo=github&style=flat-square)
[![Crates.io](https://img.shields.io/crates/v/icm42688?color=C96329&logo=Rust&style=flat-square)](https://crates.io/crates/icm42688)
[![docs.rs](https://img.shields.io/docsrs/icm42688?color=C96329&logo=rust&style=flat-square)](https://docs.rs/icm42688)
![MSRV](https://img.shields.io/badge/MSRV-1.56-blue?style=flat-square)
![Crates.io](https://img.shields.io/crates/l/icm42688?style=flat-square)

An `embedded-hal` driver for the ICM-42688 6-axis IMU.

While this device supports communication via I²C, SPI, and I3C, presently only I²C is supported. In its current state we are able to read the accelerometer, gyroscope, and temperature sensor data and perform basic configuration of the device. Reading packets from the FIFO is not currently supported.

If there is a feature which has not yet been implemented and which you are interested in, please feel free to open an issue and/or a pull request!

## Examples

Examples can be found in the examples directory

## Resources

- [ICM-42688 Product Page](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-P/)
- [ICM-42688 Datasheet](https://DS-000347-ICM-42688-P-v1.0.pdf)

## License

Licensed under either of:

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in
the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without
any additional terms or conditions.
