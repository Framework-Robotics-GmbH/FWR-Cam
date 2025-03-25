# FWR-Cam_lnx Library

This project provides a common C++ abstraction base class for V4L2 camera
devices, along with extending base classes for specific camera models for more
settings (e.g. HIDraw-based).

It is designed to simplify interfacing with V4L2 cameras in C++ projects,
streamlining camera discovery, control, and settings management. WIP

## Features
- Common C++ base class for generic V4L2 camera handling
- Specific base classes for specific camera models

## License
Licensed under the BSD 3-Clause License.  
See the [LICENSE](LICENSE) file for details.

## Maintainer
This project is developed and maintained by [Framework Robotics GmbH](https://fw-robotics.de).

## How to Contribute
Contributions are welcome!  
Please read [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Build instructions
```bash
cmake --preset=default
cmake --build --preset=default
```

### Install (optional)
```bash
cmake --install build
```

### Package (optional)
Produces tarball. (Also builds. You can ommit the build step above.)
```bash
cmake --build --preset=default --target package
```

Requires a C++ compiler with C++20 support,  CMake 3.19 or newer, and Linux
kernel headers.

## Documentation
Documentation is currently minimal; contributions welcome!  
Example usage and interface documentation are planned.

## Contact
For questions or collaboration inquiries, please contact: info@fw-robotics.de
