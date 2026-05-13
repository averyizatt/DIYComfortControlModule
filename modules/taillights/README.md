# Taillights Module (external repo placeholder)

Add your custom taillights firmware repository at this path (submodule or vendored copy) so it remains an independent PlatformIO project.

Expected behavior:
- It uses the shared CAN contract in `shared/can_contract/include/can_contract/can_protocol.h`.
- It publishes/consumes taillight CAN IDs from the shared contract.
- It is built/flashed with its own PlatformIO environments inside that repo.
