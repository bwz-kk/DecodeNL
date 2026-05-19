# DecodeNL — Agent Guidelines

This document governs how AI agents and developers contribute to the DecodeNL FTC robotics repository.  
All contributors — human or automated — **must** follow these rules.

---

## 1. Workflow Rules

### 1.1 Read Before You Write

- Before modifying any file, **read all existing documentation** (`AGENTS.md`, `FEATURES.md`, any `readme.md`, and relevant `*Constants.java` files) that describe the system you are changing.
- Before adding a new subsystem, command, or hardware wrapper, **read every file in the corresponding directory** to understand existing patterns.
- Do not assume file contents based on structure alone — always verify with a read.

### 1.2 Explain Planned Changes

- Before implementing any non-trivial change, **describe your plan** in terms of:
  - What feature or bug is being addressed.
  - Which subsystems or classes will be modified or created.
  - How the change integrates with existing systems (drivetrain, vision, turret, intake, gate, hardware abstraction).
  - Which constants or configuration values will be added or changed.
- Use this description to confirm correctness with the team before writing code.

### 1.3 Document the Implementation

- Every new class, method, or significant behavior change **must** be recorded in `FEATURES.md` (or the appropriate documentation file) immediately after implementation.
- Documentation entries must describe:
  - **Purpose:** Why this exists.
  - **Behavior:** How it behaves at runtime.
  - **Integrations:** Which other systems it interacts with.
  - **Responsibilities:** What guarantees it provides.

### 1.4 Update Related Docs

- When changing a feature, update **all** documentation files that reference that feature.
- When renaming, moving, or deleting a class, update `FEATURES.md` and any `readme.md` files that reference it.
- When changing constants (hardware map names, tuning values, calibration data), verify that `FEATURES.md` and related docs still describe the correct behavior — even if the constants themselves are not documented inline.

### 1.5 Avoid Duplicate Logic

- Before writing new code, search the repository for existing implementations of similar functionality. Use `search_files` with appropriate regex patterns.
- If similar functionality exists in a different subsystem, **reuse or generalize** rather than duplicate.
- Common candidates for reuse:
  - PID control logic (use `Controller/PIDFController.java` or existing `PIDController` from SolversLib).
  - Servo wrappers with analog feedback (use `Hardware/TauraServo.java`).
  - Kalman filtering (use `Subsystem/Vision/KalmanFilter1D.java` — compose multiple instances for multi-dimensional state).
  - Path-following command scheduling (use `FollowPathCommand` from SolversLib).
- Any code considered for duplication must first be discussed with the team.

---

## 2. Documentation-First Development

### 2.1 Documentation Before Code

1. Before creating a new subsystem, first author a section in `FEATURES.md` describing its purpose, behavior, integrations, and responsibilities.
2. Have the documentation reviewed before writing any Java code.
3. Only after documentation is approved, proceed to implement the subsystem.

### 2.2 Documentation After Changes

- Every pull request or completed task **must** include documentation updates alongside code changes.
- A PR that adds code without corresponding documentation updates **will not be merged**.

### 2.3 Source of Truth

- `FEATURES.md` is the canonical reference for what the robot does.
- `AGENTS.md` is the canonical reference for how to work on the robot.
- `*Constants.java` files are the canonical reference for hardware map names, tuning values, and calibration data.
- In any conflict between code and documentation, **the code is assumed correct** but the documentation must be updated to match.

---

## 3. Feature-Based Architecture Rules

### 3.1 Subsystem Independence

- Each subsystem (`Subsystem/<SubsystemName>/`) owns its hardware, its state, and its `periodic()` behavior.
- Subsystems **must not** directly access another subsystem's hardware. Use commands or a shared data class if cross-subsystem communication is needed.
- Subsystems expose behavior through public methods; they do not expose raw hardware objects.

### 3.2 Command Encapsulation

- All subsystem actions are invoked through commands (`Commands/<SubsystemName>/`).
- Commands call exactly one public method on the subsystem in their `initialize()` and then finish immediately (`isFinished() == true`) for instantaneous actions.
- For continuous actions (e.g., holding a button), bind commands to `whenActive` / `whenInactive` button events in the OpMode.

### 3.3 Constants Isolation

- Each subsystem has a `*Constants.java` file containing:
  - Hardware map names (`public static final String HM...`).
  - Tuning values (power, position, PID gains).
  - Any enumerations specific to that subsystem.
- Constants files are the single point of calibration. **Do not hardcode** power values, positions, or hardware names in subsystem code.

### 3.4 OpMode as Orchestrator

- OpModes (`@TeleOp`, `@Autonomous` classes) are responsible only for:
  - Creating subsystem instances.
  - Binding gamepad inputs to commands (in TeleOp).
  - Scheduling command sequences (in Autonomous).
  - Running the main loop (`super.run()` + `follower.update()` + telemetry).
- OpModes **must not** contain mechanism logic. All mechanism behavior lives in subsystems and commands.

---

## 4. Subsystem Reuse Rules

### 4.1 When to Create a New Subsystem

- Create a new subsystem only when the mechanism controls a physically distinct, independently controllable part of the robot.
- If a new mechanism is a minor variant of an existing subsystem (e.g., a second servo of the same type), add configuration to the existing subsystem rather than creating a new one.

### 4.2 Hardware Wrapper Reuse

- Always prefer using existing hardware wrappers (`TauraServo`, `PIDServo`) over directly accessing raw FTC hardware objects.
- If a new type of hardware interaction is needed (e.g., a new sensor), create the wrapper in `Hardware/` and document it in the FEATURES.md Hardware Abstraction Layer section.

### 4.3 Controller Reuse

- Always prefer using `Controller/PIDFController.java` for any custom PID control need.
- If SolversLib's built-in `PIDController` is sufficient, use that instead.
- Do not write inline PID logic in subsystem code.

---

## 5. Telemetry Standards

### 5.1 Telemetry Placement

- Subsystems report their internal state in `periodic()` using `telemetry.addData()`.
- Do **not** place telemetry calls inside commands — commands should be silent.
- OpModes report high-level data (robot pose, current action) using `TelemetryData` from SolversLib.

### 5.2 Dashboard Integration

- Use `FtcDashboard.getInstance().getTelemetry()` for dashboard-compatible telemetry in subsystems.
- Use `FtcDashboard.getInstance().sendTelemetryPacket(packet)` with `TelemetryPacket` and field overlay drawing in OpModes.
- Camera streams from Limelights are started in vision module `init()` methods using `FtcDashboard.getInstance().startCameraStream()`.

### 5.3 Telemetry Content

- Subsystem telemetry **must** include: current state/target values, any sensor feedback readings, and error/status indicators.
- Subsystem telemetry **should not** include: debug information not relevant to tuning or match operation.
- OpMode telemetry **must** include: robot pose (X, Y, heading), current autonomous step (if in auto), and key mechanism statuses.

---

## 6. Implementation Approval Flow

For any change beyond trivial bug fixes (e.g., adding a new subsystem, modifying autonomous behavior, changing vision pipeline):

### 6.1 Step 1: Proposal
- Write or update the relevant section in `FEATURES.md` describing the proposed behavior.
- List all files that will be created or modified.
- Describe integration points with existing systems.

### 6.2 Step 2: Review
- Present the proposal to the team for review.
- Address questions about design, calibration needs, and testing plan.

### 6.3 Step 3: Implementation
- Implement the code following the documented design.
- Run `./gradlew :TeamCode:assembleDebug` to confirm the project builds.

### 6.4 Step 4: Documentation Update
- Update `FEATURES.md` with any implementation details that diverged from the proposal.
- Ensure `*Constants.java` files reflect the final calibration values.

### 6.5 Step 5: Test
- Run `./gradlew :TeamCode:assembleDebug` again.
- Validate on the robot or FTC-approved test setup.
- Record the OpMode, mechanism tested, and any known limitations.

### 6.6 Step 6: Commit
- Write a concise commit summary that includes validation status (e.g., "turret auto-aim (tested)", "vision odometry fusion (untested)").
- Include references to any relevant documentation changes.

---

## 7. Markdown Documentation Requirements

### 7.1 File Locations

| File | Location | Content |
|------|----------|---------|
| Feature documentation | `FEATURES.md` (repo root) | Purpose, behavior, integrations, responsibilities for every feature |
| Agent guidelines | `AGENTS.md` (repo root) | Workflow rules, architecture rules, standards |
| Subsystem notes | `Subsystem/<Name>/readme.md` (optional) | Per-subsystem tuning notes, calibration procedures |
| Sample documentation | `samples/` + OpMode Javadoc | How to use the sample, what it demonstrates |

### 7.2 Format Rules

- Use Markdown heading hierarchy: `#` for document title, `##` for major sections, `###` for subsections, `####` for sub-subsections.
- Use `**bold**` for terminology and subsystem names.
- Use `inline code` for class names, method names, constants, and file paths.
- Use fenced code blocks (```) for code snippets and configuration examples.
- Use `---` (horizontal rule) between major sections for readability.
- Use bullet lists (`-`) for unordered items, numbered lists for sequential steps.
- Keep each file focused: `FEATURES.md` describes **what**, `AGENTS.md` describes **how**, code describes **implementation**.

### 7.3 Consistency Requirements

- When a subsystem or feature is renamed in code, update its name in `FEATURES.md` and any cross-references within 24 hours.
- When a hardware map name changes in `*Constants.java`, verify that any documentation referencing the old name is updated.
- When a feature is removed, mark it as **Deprecated** in `FEATURES.md` for one week before removing the documentation entry, to give the team time to react.

---

## 8. AI-Specific Rules

### 8.1 Pre-Work Verification

Before writing any code, the AI must:
1. Read `AGENTS.md` (this file) to confirm current rules.
2. Read `FEATURES.md` to understand the feature being modified.
3. Read the relevant subsystem and command source files to understand existing patterns.
4. Read the relevant `*Constants.java` file to understand current calibration values.

### 8.2 Change Explanation

Before writing code, the AI must explain:
- Which files will be created, modified, or deleted.
- Which existing patterns are being followed.
- How the change integrates with each existing subsystem it touches.

### 8.3 Post-Work Documentation

After writing code, the AI must:
1. Update `FEATURES.md` to reflect any new or changed features.
2. If a new subsystem was created, add its section to `FEATURES.md`.
3. If hardware map names or tuning values changed, confirm `FEATURES.md` remains accurate in its behavioral descriptions.
4. Report any deviations from the plan that occurred during implementation.
5. Run `./gradlew :TeamCode:assembleDebug` to confirm the project builds.

### 8.4 Duplicate Prevention

- Before adding a new class, the AI **must** search the repository for existing implementations with similar functionality.
- If a match is found, the AI must either reuse the existing implementation or explain why it is insufficient.
- The AI must not create a second PID controller, a second servo wrapper, or a second Kalman filter implementation.