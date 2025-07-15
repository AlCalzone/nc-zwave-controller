// @ts-check
import fs from "node:fs/promises";
import path from "node:path";

const versionFile = await fs.readFile(
  "build/config/zw_version_config.h",
  "utf8"
);
const majorVersion = versionFile.match(/\bUSER_APP_VERSION\s+(\d+)/)?.[1];
const minorVersion = versionFile.match(/\bUSER_APP_REVISION\s+(\d+)/)?.[1];
const patchVersion = versionFile.match(/\bUSER_APP_PATCH\s+(\d+)/)?.[1];

if (!majorVersion || !minorVersion || !patchVersion) {
  console.error(
    "Failed to extract version information from zw_version_config.h"
  );
  process.exit(1);
}

console.log(`${majorVersion}.${minorVersion}.${patchVersion}`);
