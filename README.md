# W25 Flash driver

[![crates.io](https://img.shields.io/crates/v/w25.svg)](https://crates.io/crates/w25) [![Documentation](https://docs.rs/w25/badge.svg)](https://docs.rs/w25)

This is a generic async driver for the W25 flash chips from Winbond.

Supported series:
- Q
- X

More series support is welcomed!

Defmt is also supported through the `defmt` feature.

This crate is adopted from <https://github.com/tweedegolf/w25q32jv>

## TODO

- Fast read support. So far there's only support for the normal read, so don't use a SPI speed of > 50Mhz

## Changelog

### Unreleased

- Added fetching JEDEC ID & constructor which autodetects the chip capacity.

### 0.7.0 2026-07-07

- Added a no-pin constructor
- *breaking*: Removed the pin error variant from the error enum

### 0.6.0 2025-07-07

- Continued from <https://github.com/tweedegolf/w25q32jv> 0.5.1
- Driver and driver struct has been renamed to W25
- Series is now a generic so the driver can support multiple series
- The capacity is now given at runtime so we don't need feature flags
