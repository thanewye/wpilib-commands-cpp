# WPILib Commands C++ (Java-ordered scheduler)

A fork of the WPILib 2026.2.1 C++ command-based library (`frc2`). Its `CommandScheduler` iterates and defers work in the same order as the Java `CommandScheduler`. Every header is identical to upstream, so libraries compiled against `wpilibNewCommands` headers (for example ChoreoLib) link against this unchanged.

This is a personal project. It is not affiliated with or endorsed by WPI or FIRST.

## Why

Upstream C++ stores scheduler state in pointer-keyed hash containers, so the order of subsystem `Periodic`, command `Execute` and default-command scheduling depends on heap addresses and can change between runs of the same binary. Java uses insertion-ordered containers. Robot code that is replayed with AdvantageKit and compared against a Java implementation diverges whenever two subsystems or commands interact within one cycle.

## Behavior differences from upstream C++

These all match Java `CommandScheduler`:

- Subsystem `Periodic` and `SimulationPeriodic` run in registration order.
- Commands `Execute` in the order they were scheduled.
- Default commands are scheduled in subsystem registration order.
- An incoming command interrupts conflicting commands in the order of its own requirements.
- `Schedule` and `Cancel` calls made while the scheduler is iterating commands are deferred until the loop ends; then the queued schedules run, followed by the queued cancels.
- A command stays scheduled, and keeps its requirements, until its `End` returns.
- `Cancel` on a command that is already ending does nothing.

Everything outside `CommandScheduler.cpp` is upstream source.

## Build and publish

```bash
./gradlew build
./gradlew clean build publish -PreleaseMode \
  -PpublishVersion=2026.2.1-1 \
  -PpublishGroup=com.example.wpilibcommands \
  -PvendordepMavenUrl=https://example.github.io/wpilib-commands-cpp/maven \
  -PvendordepJsonUrl=https://example.github.io/wpilib-commands-cpp/WPILibCommandsCpp.json
```

The Maven repository is written to `build/repos/releases` and the expanded vendordep to `build/vendordep/WPILibCommandsCpp.json`. See [PUBLISHING.md](PUBLISHING.md).

## Use in robot code

Delete `vendordeps/WPILibNewCommands.json` and install `WPILibCommandsCpp.json` in its place. The `frc2` includes and API stay the same. Keeping both vendordeps links two copies of `frc2` and is not supported.

The vendordep ships only the C++ library. Java robot projects should keep using WPILib's own `WPILibNewCommands.json`.

## License

The command library is WPILib code under the WPILib BSD license (`WPILIB-LICENSE.md`). Build and publishing scripts are MIT (`LICENSE`). See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
