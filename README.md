# LouCOMAX_Controllers

Controllers for hardware in the LouCOMAX Project.

## Overview

LouCOMAX_Controllers is a collection of software modules designed to interface with and control various hardware components used in the LouCOMAX Project. The controllers provide reliable communication, automation, and monitoring for supported devices.

The following hardware devices are interfaced within this package :
- Amptek DP5, PX5, DP5X, DP5G, MCA8000D, Mini-X2 and products derived from these (X123, X55, Gamma-Rad5, TB-5).
- iMOXS/2® Control Service Unit for X-Ray Source
- Specific setup of Arduino with telemetry laser : HG-C1100-P
- Thorlabs' BCS203 with NanoMax300 micropositioning stages
- Zaber multi stage daisy chain (contains 3 stages XYZ)

Note : The modules were initialy developped for specific hardware setup on LouCOMAX project.
It may not be usable as is for some setup too different. Please feel free to contribute to the 
github or ask questions to developper.
 
## Features

- Modular controller architecture for easy extension
- Support for multiple hardware interfaces (e.g., serial, USB, Ethernet)
- Logging and error handling
- Example scripts for common hardware tasks

## Getting Started

1. **Clone the repository:**
   ```
   git clone https://github.com/abpydev/LouCOMAX_Controllers.git
   ```
2. **Install dependencies:**  
   Refer to the `requirements.txt` or relevant setup instructions for your platform.

3. **Run an example controller:**
   ```
   python controllers/example_controller.py
   ```

## Directory Structure

```
LouCOMAX_Controllers/
├── controllers/         # Main controller modules
├── examples/            # Example usage scripts
├── tests/               # Unit tests
├── README.md            # Project documentation
└── requirements.txt     # Python dependencies
```

## Initial developpers

Developped by : 

- Antoine BLASIAK : antoine.blasiak66@gmail.com

Other contributors :

- Laurent PICHON : laurent.pichon@culture.gouv.fr

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## License

This project is licensed under the MIT License.
