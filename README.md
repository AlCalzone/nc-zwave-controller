# Z-Wave Controller firmware

## Firmware versioning scheme

The versioning for the firmware follows the format `A.BBC`, where:

- `A = 1` is the hardware/device revision
- `BB` is a number (0-99) that represents the Z-Wave SDK version the firmware was built with
- `C` is the revision (0-9) of our modifications on top of the SDK

### SDK version mapping

| `BB` | Simplicity SDK version         | Z-Wave SDK version     |
| ---- | ------------------------------ | ---------------------- |
| `0`  | [`2024.12.1`][sisdk-2024.12.1] | [`7.23.1`][zdk-7.23.1] |

<!-- SDK links -->
[sisdk-2024.12.1]: https://github.com/SiliconLabs/simplicity_sdk/releases/tag/v2024.12.1-0
[zdk-7.23.1]: https://www.silabs.com/documents/public/release-notes/SRN14930-7.23.1.0.pdf
