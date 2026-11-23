# Historical deployment overlay — not a customer release

This opt-in overlay preserves the launch/config/test files from candidate commit d5d5927 for existing deployments. It contains original deployment identifiers intentionally. It is excluded from all customer ZIPs and is not installed by the normal build. Do not copy it into new customer workspaces.

For an existing deployment only, copy `ros2-legacy-overlay/lucsys_chassis_driver/` over a separate copy of the matching driver package, then rebuild and run its launch contract tests. Explicit ports take priority. Model aliases only translate names; they do not establish hardware compatibility. This overlay does not supply external sensor drivers. Keep it out of customer distribution archives.
