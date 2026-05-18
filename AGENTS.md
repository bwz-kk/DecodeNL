# Repository Guidelines

## Project Structure & Module Organization

This is an FTC Android Gradle project. `TeamCode/` contains team-owned robot code and is the primary place for changes. Its Java package root is `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`, with modules such as `Commands/`, `Subsystem/`, `Hardware/`, `Field/`, `Controller/`, `pedroPathing/`, and `samples/`. Team resources live under `TeamCode/src/main/res/`.

`FtcRobotController/` contains the upstream FTC Robot Controller app and bundled sample OpModes. Avoid editing SDK files unless the change is about app integration. Shared Gradle configuration is in `build.common.gradle`; documentation and legal assets are under `doc/`.

## Build, Test, and Development Commands

- `./gradlew assembleDebug`: builds debug APKs for the FTC app and `TeamCode`.
- `./gradlew :TeamCode:assembleDebug`: builds only the team module and its required dependencies.
- `./gradlew lint`: runs Android lint checks when available.
- `./gradlew clean`: removes generated build outputs before a fresh build.

Open the project in Android Studio Ladybug or later for deployment to a Robot Controller device. Use the Gradle wrapper included in this repository instead of a system Gradle install.

## Coding Style & Naming Conventions

Use Java 8-compatible code. Keep package names aligned with `org.firstinspires.ftc.teamcode`. Use PascalCase for classes (`TurretVision`, `PIDServo`), camelCase for methods and fields, and UPPER_SNAKE_CASE for true constants. Existing subsystem folders use names such as `Subsystem/Intake`; follow nearby conventions when adding related files.

Keep hardware-map names and tuning values in `*Constants` classes where practical. Do not place robot-specific logic in `FtcRobotController` samples.

## Testing Guidelines

There are currently no dedicated unit test source sets. Before opening a PR, run at least `./gradlew :TeamCode:assembleDebug`. For behavior changes, validate on the robot or FTC-approved test setup and record the OpMode, mechanism, and known limitations. Name future tests after the class or behavior under test, for example `PIDFControllerTest`.

## Commit & Pull Request Guidelines

Recent commits use short, descriptive summaries such as `vision (tested)` and `robot draw and turret vision (untested)`. Keep the first line concise and include validation status when relevant.

Pull requests should describe the robot behavior changed, list build or hardware tests run, link related issues/tasks, and include screenshots or dashboard captures for vision, telemetry, or UI changes. Call out any required robot configuration, calibration, or dependency changes.

## Agent-Specific Instructions

Preserve upstream FTC SDK structure. Prefer scoped edits in `TeamCode/`, avoid broad formatting churn, and do not modify generated build outputs or keystore files.
