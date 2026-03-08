<!-- Copyright (c) 2025-2026 Webb Robotics -->
<!-- http://github.com/FRC1466 -->

# REBUILT Capability Review and Improvement Roadmap

This document ties the current robot code to the 2026 FRC REBUILT game model, summarizes what the robot already does, and prioritizes the additions most likely to improve match results with the least extra complexity.

## 1. What the robot already does well

### Scoring and shooting

- **Distance-based hub shooting is implemented.**
  - `ShotCalculator` calculates hood angle, flywheel speed, time of flight, and drive heading from robot pose with lookahead for robot motion.
  - File: `src/main/java/frc/robot/subsystems/shooter/ShotCalculator.java`
- **A coordinated scoring state machine already exists.**
  - `Choreographer` handles `SCORE_HUB` by spinning the flywheel, moving the hood, waiting for drivetrain alignment, then running the indexer and kicker.
  - File: `src/main/java/frc/robot/subsystems/Choreographer.java`
- **Preset and fallback shot behavior already exists.**
  - Right bumper toggles a hub-preset override, and POV up/down adjusts flywheel speed offset for quick shot compensation.
  - Files:
    - `src/main/java/frc/robot/RobotContainer.java`
    - `src/main/java/frc/robot/subsystems/shooter/ShotCalculator.java`

### Intake and piece handling

- **The robot has a deploy/stow intake and intake rollers.**
  - `Intake` controls the pivot and rollers.
  - File: `src/main/java/frc/robot/subsystems/intake/Intake.java`
- **The indexing/feed pipeline exists in software.**
  - `Indexer` supports stall detection and recovery.
  - `Kicker` feeds into the shooter.
  - Files:
    - `src/main/java/frc/robot/subsystems/indexer/Indexer.java`
    - `src/main/java/frc/robot/subsystems/kicker/Kicker.java`
- **Important current limitation:** the physical indexer is still commented out on `COMPBOT` and `DEVBOT`, so the scoring pipeline is not fully enabled on the real robots right now.
  - File: `src/main/java/frc/robot/RobotContainer.java`

### Driver assist and field interaction

- **Launch-lock aiming already exists.**
  - While the driver is trying to shoot, `DriveCommands` can auto-rotate to the shot heading and limit translation so the robot does not move too fast for the shot solution.
  - File: `src/main/java/frc/robot/commands/DriveCommands.java`
- **The code already understands trench and bump geometry.**
  - There are trench and bump zone checks plus heading-lock behaviors for those zones.
  - Files:
    - `src/main/java/frc/robot/commands/DriveCommands.java`
    - `src/main/java/frc/robot/FieldConstants.java`
- **Vision pose fusion is implemented.**
  - `Vision` accepts AprilTag-based observations, filters them, and feeds pose updates into the drivetrain estimator.
  - It also has a useful `rampMode` concept to suppress bad updates while crossing the bump/ramp and then aggressively correct afterward.
  - Files:
    - `src/main/java/frc/robot/subsystems/vision/Vision.java`
    - `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`

### Autonomous infrastructure

- **You already have a good auto framework.**
  - `Autos` uses ChoreoLib trigger-based routines and already documents how to build split and branching autos.
  - File: `src/main/java/frc/robot/Autos.java`
- **A working depot-to-score auto exists.**
  - `depotAuto()` drives to depot, waits for load, then returns to the score location and shoots.
  - File: `src/main/java/frc/robot/Autos.java`
- **Dashboard auto selection and bring-up tests are wired.**
  - File: `src/main/java/frc/robot/RobotContainer.java`

## 2. What is incomplete or strategically underused

### Gaps already visible in the code

1. **Passing is scaffolded but not tuned.**
   - Passing maps are placeholder zeros.
   - This means the codebase is structurally ready for advanced cycling or alliance-feed plays, but not match-ready yet.
   - File: `src/main/java/frc/robot/subsystems/shooter/ShotCalculator.java`

2. **Climb goals exist without a climb subsystem.**
   - `CLIMB_EXTEND` and `CLIMB_RETRACT` are defined in `Choreographer`, but they only stop other mechanisms today.
   - File: `src/main/java/frc/robot/subsystems/Choreographer.java`

3. **Real-robot piece flow is not fully enabled.**
   - Because the indexer is commented out on the real robots, your best existing superstructure sequence is partially disabled in the most important environment.
   - File: `src/main/java/frc/robot/RobotContainer.java`

4. **You only have one real match auto.**
   - The auto framework is much more advanced than the current auto library.
   - Files:
     - `src/main/java/frc/robot/Autos.java`
     - `src/main/java/frc/robot/RobotContainer.java`

5. **Driver controls still have unused or under-leveraged buttons.**
   - `A` is effectively just a drive launch-mode hold and is a good candidate for a new assist feature.
   - File: `src/main/java/frc/robot/RobotContainer.java`

## 3. How this maps to REBUILT strategy

The official REBUILT season resources indicate that the game rewards fast autonomous execution, hub scoring throughput, intelligent navigation around central field constraints such as bumps/trenches/hubs, and endgame tower performance. Always confirm the latest values and constraints with the current game manual, Team Updates, and Q&A before lock-in:

- Official game manual: `https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf`
- Season materials: `https://www.firstinspires.org/resources/library/frc/season-materials?view=calendar`
- FIRST game page: `https://www.firstinspires.org/programs/frc/game-and-season`

### Strategic implications for this codebase

- **Your robot is already pointed toward a fast shooter-cycler role.**
  - The shot calculator, hood, flywheel, launch lock, and depot auto all support that identity.
- **The field-aware drive logic suggests you want controlled movement through constrained areas.**
  - Trench lock, bump lock, and ramp-aware vision support consistent pathing rather than freehand driving.
- **`HubShiftUtil` means you are already thinking about a hub-active / hub-inactive scoring cadence.**
  - That makes driver timing and dashboard feedback especially important.
- **The missing climb implementation is the biggest strategic hole if endgame matters heavily.**
  - If REBUILT ranking or playoff match swings depend on tower points, this gap can cap your ceiling even if teleop cycles are strong.

## 4. Highest-value improvements, in priority order

## Priority 0: finish the robot you already designed

These are the best “make us better easier” tasks because the software structure already exists.

### A. Re-enable and validate the real indexer path

Why it matters:

- This unlocks the full `Choreographer` scoring sequence on the real robot.
- It reduces driver burden by letting the robot decide when to feed only after hood/flywheel/alignment are ready.

Recommended work:

1. Enable `IndexerIOTalonFX` on `COMPBOT` and `DEVBOT`.
2. Add bring-up verification for:
   - forward feed
   - jam detection
   - recovery timing
   - handoff from intake to kicker
3. Log “piece entered / piece exited / feeding now” signals if sensors or inferred current signatures are available.

### B. Expand the auto library immediately

Why it matters:

- You already have the framework, but only one real scoring auto.
- Extra autos are one of the fastest ways to increase qualification consistency and alliance value.

Recommended first autos:

1. **Safe preload score auto**
   - Lowest-risk autonomous option for qualifications.
2. **Fast depot score auto**
   - A tightened version of the existing depot routine with shorter waits and better shot timing.
3. **Two-segment score-collect-score auto**
   - Use the split/branching structure already documented in `Autos`.
4. **Mobility / fail-safe auto**
   - For bad mechanism days or partner conflicts.
5. **Recovery auto branch**
   - If acquisition fails, pathfind to a recovery pose instead of dead-ending the routine.

### C. Finish passing mode

Why it matters:

- Passing lets you reduce long protected cycles, improve alliance coordination, and keep scoring even when primary hub lanes are congested.
- The code already contains passing detection and passing target logic.

Recommended work:

1. Replace the zeroed passing maps with real hood, flywheel, and time-of-flight values.
2. Add a driver control or auto trigger for explicit pass mode.
3. Create one practiced “safe pass lane” and one “panic clear” pass.

### D. Add operator-visible match-state feedback

Why it matters:

- If hub timing or field-state windows matter, the driver should not have to infer them mentally.
- Your code already logs enough state to expose this.

Recommended work:

1. Put the following on the dashboard:
   - hub active/inactive
   - ready-to-shoot
   - drive aligned
   - shot distance
   - passing mode active
2. Add controller rumble or a visible indicator when:
   - shot is ready
   - hub becomes active
   - endgame threshold is reached

## Priority 1: raise cycle consistency and reduce driver workload

### A. Convert left trigger into a true intake state

Right now left trigger only runs the rollers. A stronger match behavior would be:

1. deploy intake if needed
2. run rollers
3. run indexer if piece transfer is needed
4. stow automatically when the piece is secured

That turns one button into a complete acquisition action.

### B. Give `A` a dedicated assist role

Good options:

- **Vision-only align without shooting**
  - Useful for lining up before the driver commits to score.
- **Tower face / endgame approach assist**
  - Useful if you later add climb.
- **Pose snap to preset shot locations**
  - Hub, trench, outpost, depot exit.

### C. Add “score when ready” latching

Current right trigger behavior is already strong, but a latched mode can make it better:

- one press = enter scoring state
- robot automatically spins, aligns, and fires when legal/ready
- second press or cancel = return to idle

This helps under defense because the driver can focus on positioning instead of perfect hold timing.

### D. Use ramp mode actively

`Vision.setRampMode()` is useful but not obviously tied to a driver workflow yet.

Recommended uses:

- enable it automatically when entering bump-crossing assist mode
- enable it during certain autos that cross the bump
- disable it once the robot settles and let the aggressive correction snap pose back

## Priority 2: build match-winning strategic depth

### A. Add branching autos instead of only fixed autos

You already documented the pattern in `Autos`. Use it for:

- “if piece acquired, go score”
- “if missed, go depot”
- “if partner starts in our preferred lane, use alternate exit”

This will make your autonomous plan more resilient at events.

### B. Add shot quality analytics

Track:

- distance at shot
- drive alignment error
- hood error
- flywheel error
- whether the shot happened from hub preset override
- whether the shot was made or missed if you can infer it

This gives you a fast path to tuning between matches.

### C. Implement endgame/climb only after scoring reliability is solid

The codebase already hints at future climb goals, but the smarter build order is:

1. finish piece flow
2. finish autos
3. finish pass mode
4. only then spend major time on climb unless REBUILT scoring makes climb absolutely mandatory

If climb points or ranking points are dominant at your events, move this up immediately.

## 5. Concrete auto ideas that fit your current architecture

These all fit the current Choreo + trigger-based `Autos` design.

### Auto 1: Safe score and settle

- reset odometry
- drive to a stable shooting pose
- set `SCORE_HUB`
- wait for `isReadyToShoot()`
- fire
- either hold pose or leave to clear traffic

Best use:

- qualifications
- alliance partners with conflicting paths
- low-confidence intake days

### Auto 2: Depot cycle

- preload score
- drive to depot
- collect or wait for load
- drive back to score
- shoot again

This is the natural evolution of the existing depot auto.

### Auto 3: Score, collect, branch

- first score
- drive through collection segment with intake active
- if piece secured, go to score segment
- else go to depot or safe park segment

This makes direct use of the branching pattern already documented in `Autos`.

### Auto 4: Bump-crossing recovery auto

- run a path that uses bump/trench geometry intentionally
- enable vision ramp mode during crossing
- reacquire pose after crossing
- finish with a safe shot or staging pose

This is valuable because you already have both bump-aware drive logic and ramp-aware vision logic.

## 6. Recommended driver-feature roadmap

If the goal is “perform better with less effort,” these are the best additions:

1. **Piece-secured automation**
   - one-button intake behavior
2. **Ready-to-shoot feedback**
   - rumble or dashboard light
3. **Vision-only align button**
   - helps driver stage before committing
4. **Latched score mode**
   - reduces hold precision requirements
5. **Preset pose snap / shot family select**
   - trench, tower, outpost, depot return
6. **Endgame reminder / tower-approach assist**
   - only after climb exists

## 7. Suggested build order for the next month

### Week 1

- Re-enable indexer on real robot
- Validate complete intake-to-shot flow
- Add ready-to-shoot dashboard/rumble feedback

### Week 2

- Add two more autos
- Practice with Choreo event markers for intake/spin-up timing
- Add a fail-safe mobility auto

### Week 3

- Tune passing maps
- Add a dedicated pass control path
- Add shot analytics logs for post-match review

### Week 4

- Add one advanced assist feature (`A` button)
- Add one branching auto
- Decide whether climb moves ahead of more shooter/cycle refinement

## 8. Bottom line

The robot code already supports a strong REBUILT identity: **pose-aware shooter, guided driver aiming, field-aware driving, and Choreo-based autonomous sequencing**.

The fastest path to better event performance is not a giant rewrite. It is:

1. **finish the real scoring pipeline**
2. **add more autos**
3. **turn passing from scaffold into a real option**
4. **reduce driver thinking with clearer assist features**
5. **only then expand into deeper strategic features like climb or full branching autonomy**

That path gives the biggest competitive return for the least extra complexity.
