# Publishing the vendordep

A release consists of immutable Maven artifacts and a `WPILibCommandsCpp.json` vendordep whose coordinates and platform list match those artifacts.

## Versioning

Versions are `<WPILib version>-<fork revision>`, for example `2027.0.0-alpha-7-1`. Rebase onto a new WPILib release by replacing `src/main/native/{cpp,include}` with that tag's `commandsv2` sources, generated sources and headers, then reapplying the `CommandScheduler.cpp` changes.

## Before the first release

1. Keep the UUID in `WPILibCommandsCpp.json` stable across all releases.
2. Keep the WPILib year, GradleRIO version and CI images aligned with the forked tag.
3. Host the Maven repository and vendordep JSON over anonymous HTTPS.
4. Never overwrite an existing version.

## Build a release candidate

```bash
./gradlew clean build publish -PreleaseMode \
  -PpublishVersion=2027.0.0-alpha-7-1 \
  -PpublishGroup=com.ethanye.wpilibcommands \
  -PvendordepMavenUrl=https://thanewye.github.io/wpilib-commands-cpp/maven \
  -PvendordepJsonUrl=https://thanewye.github.io/wpilib-commands-cpp/WPILibCommandsCpp-2027.json
```

The Maven repository is written to `build/repos/releases`, the expanded vendordep to `build/vendordep/WPILibCommandsCpp.json`, and the platform archives to `build/allOutputs`.

## Validate before release

1. Build every advertised platform through CI.
2. Confirm the headers are byte-identical to the upstream tag's headers, apart from `Trigger.hpp` taken from upstream `main`.
3. Install the candidate vendordep into a clean 2027 C++ robot project in place of `CommandsV2.json` and compile desktop and Systemcore targets.
4. Confirm the robot binary links `libWPILibCommandsCpp` and not `libcommandsv2`.

## Publish

Run the `Publish` workflow with a new version. It preserves existing Maven versions, updates `WPILibCommandsCpp-2027.json`, writes a versioned JSON snapshot, and deploys the combined repository through GitHub Pages.
