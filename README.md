# DIYComfortControlModule Workspace

Firmware workspace for Foxbody Mustang distributed modules.

## Workspace Layout

- `.` (root): current **comfort/master** PlatformIO firmware project
- `shared/can_contract/`: shared CAN protocol contract used by all modules
- `modules/comfort/`: comfort module metadata/schema pin
- `modules/water-meth/`: placeholder path for external water-meth repo
- `modules/taillights/`: placeholder path for external custom taillights repo

This keeps each firmware independent while sharing one CAN contract definition.

## Shared CAN Contract

Canonical CAN protocol definitions live in:

- `shared/can_contract/include/can_contract/can_protocol.h`

The comfort project consumes this through:

- `src/can/can_protocol.h` (shim include)

## Build / Flash

### Comfort module (this repo root)

```bash
pio run -e nano_esp32_debug
pio run -e nano_esp32_release
pio run -e nano_esp32_release -t upload
```

### Water-meth module (external repo in `modules/water-meth/`)

```bash
cd modules/water-meth
pio run -e <env>
pio run -e <env> -t upload
```

Upstream repo: `https://github.com/averyizatt/DIYWaterMethInjection`

### Taillights module (external repo in `modules/taillights/`)

```bash
cd modules/taillights
pio run -e <env>
pio run -e <env> -t upload
```

Upstream repo: `https://github.com/averyizatt/CustomTaillights`

## Lightweight Integration Check

Run from workspace root:

```bash
python scripts/check_can_contract.py
```

The check verifies:

- shared contract contains required taillight + meth IDs
- comfort shim includes shared contract header
- each module schema pin (`modules/*/CAN_PROTOCOL_SCHEMA_VERSION`) matches `CAN_PROTOCOL_SCHEMA_VERSION`

## External Repo Setup

Place your other firmware repositories at:

- `modules/water-meth/`
- `modules/taillights/`

You can use either submodules or vendored copies.

Recommended submodule commands:

```bash
git submodule add https://github.com/averyizatt/DIYWaterMethInjection modules/water-meth
git submodule add https://github.com/averyizatt/CustomTaillights modules/taillights
git submodule update --init --recursive
```
