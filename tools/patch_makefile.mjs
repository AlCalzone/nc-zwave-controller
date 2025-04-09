// @ts-check
import fs from "node:fs/promises";
import path from "node:path";

const SDK_DIR = "simplicity_sdk_2024.12.2";

const filename = (await fs.readdir("build")).find((f) => f.endsWith(".mak"));
if (!filename) {
  console.error("Makefile not found in build directory.");
  process.exit(1);
}

let makefile = await fs.readFile(`build/${filename}`, "utf8");

let prefixEnd = makefile.indexOf("SDK Build Rules");
prefixEnd = makefile.indexOf("#####\n", prefixEnd) + 6;

const suffixIndex = makefile.lastIndexOf(
  "# Automatically-generated Simplicity Studio Metadata"
);

const allSources = (await fs.readdir(process.cwd(), { recursive: true }))
  .filter((f) => f.endsWith(".c") || f.endsWith(".S"))
  .filter((f) => !f.split(path.sep).includes("build"));

const projectSources = allSources.filter(
  (f) => !f.split(path.sep).includes(SDK_DIR)
);

const sdkSources = allSources.filter((f) =>
  f.split(path.sep).includes(SDK_DIR)
);

let fileImports = "";
for (const source of sdkSources) {
  let relative = source.slice(source.indexOf(path.sep) + 1);
  let withoutExt = relative.slice(0, relative.lastIndexOf("."));

  fileImports += `
$(OUTPUT_DIR)/sdk/${withoutExt}.o: $(COPIED_SDK_PATH)/${relative}
	@$(POSIX_TOOL_PATH)echo 'Building $(COPIED_SDK_PATH)/${relative}'
	@$(POSIX_TOOL_PATH)mkdir -p $(@D)
	$(ECHO)$(CC) $(CFLAGS) -c -o $@ $(COPIED_SDK_PATH)/${relative}
CDEPS += $(OUTPUT_DIR)/sdk/${withoutExt}.d
OBJS += $(OUTPUT_DIR)/sdk/${withoutExt}.o

`;
}

for (const source of projectSources) {
  let withoutExt = source.slice(0, source.lastIndexOf("."));

  fileImports += `
$(OUTPUT_DIR)/project/${withoutExt}.o: ${source}
	@$(POSIX_TOOL_PATH)echo 'Building ${source}'
	@$(POSIX_TOOL_PATH)mkdir -p $(@D)
	$(ECHO)$(CC) $(CFLAGS) -c -o $@ ${source}
CDEPS += $(OUTPUT_DIR)/project/${withoutExt}.d
OBJS += $(OUTPUT_DIR)/project/${withoutExt}.o

`;
}

makefile =
  makefile.slice(0, prefixEnd) + fileImports + makefile.slice(suffixIndex);

makefile = makefile.replace(
  /^BASE_SDK_PATH = .*$/m,
  `BASE_SDK_PATH = ${path.join(process.cwd(), SDK_DIR)}`
);

await fs.writeFile(`build/${filename}`, makefile);
console.log("Makefile patched.");
