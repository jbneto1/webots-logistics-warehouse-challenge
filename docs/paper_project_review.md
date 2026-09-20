# Review of the educational scenario, controller API, and paper

Reviewed on 11 September 2026 against the current working tree. The author confirmed `CBL___Webots___RAF___ROBOT_2026/conference_101719.tex` as the manuscript to review.

The project has a coherent teaching structure: a rule supervisor, a robot policy, reusable motion actions, and explicit logistics events. The central weakness in the paper is that it inventories these components more thoroughly than it explains the decisions a student makes with them. The overlay has the same problem: it exposes implementation values without consistently translating them into task meaning.

This is a source and screenshot review, not an executed Webots benchmark or a classroom evaluation. Findings below distinguish implemented behavior, inferred failure paths, and proposed changes. Neither the manuscript nor the implementation was edited. Existing local changes to map markers and the world project settings were preserved.

The adjacent `conference_101719.pdf` is the three-page IEEE template, not a compiled version of the confirmed manuscript. The two PDFs in `output/pdf` are different drafts. Consequently, observations about current figures use their image assets and LaTeX definitions; current compiled pagination and float placement have not been verified.

## 1. Explain readiness as a task concept before introducing the mask

**Priority: high. This is both a paper issue and an interface issue.**

The manuscript says that bit `i` marks a processed part. This omits the distinction between a completed processing stage, an output waiting for collection, and a machine available to receive work. A box that has already been collected has completed processing but no longer has its ready bit set.

There are three different identifiers:

| Identifier | Current domain | Meaning |
| --- | --- | --- |
| Machine | `A` or `B` | Processing stage |
| Box index `i` | `0`–`3` | Persistent identity of `BOX_i` |
| Bay index `b` | `0`–`1` within each machine | Input/output lane used by that machine |

The ready mask is indexed by **box identity**, not bay identity. For machine `M`, its value is the sum of `2^i` for boxes whose processed outputs are waiting for collection there. Bit 0 is the least significant bit.

| Decimal mask | Binary, ordered BOX_3 … BOX_0 | Interpretation |
| --- | --- | --- |
| `0` | `0000` | No processed output is waiting for collection at this machine |
| `1` | `0001` | BOX_0 is waiting |
| `4` | `0100` | BOX_2 is waiting |
| `5` | `0101` | BOX_0 and BOX_2 are waiting |

`A=5` does not mean five boxes, bay 5, or a machine-availability score. Because a machine has two output bays, at most two ready bits can normally be set for it in this world. Although four bits can encode 0–15, not all encodings are reachable machine states.

The mask alone does not identify the bay. For example, `READY A 2 1 -0.15500 -0.15000 3.14159` associates BOX_2 with Machine A, bay 1, and the **robot-center pickup pose**. The pose is not the box position and its availability does not mean the robot has reached it.

The actual lifecycle is:

1. A valid input placement is accepted and earns one point.
2. If the corresponding output is occupied, the new input waits without starting its processing timer.
3. Once that output is free, processing starts with a sampled 15–25 s duration.
4. At completion, the supervisor changes the box type/color, places it at the output, sets its ready flag, and emits `READY`.
5. Outstanding `READY` messages are repeated every supervisor step, and `POSE` carries a mask snapshot.
6. When the supervisor registers pickup, it clears the ready flag and output occupancy, emits `CLEAR`, and may immediately start a queued input.

The last transition is based on logical pickup registration, before the robot necessarily finishes reversing out of the bay. Do not describe it as continuous physical vacancy detection.

**Suggested replacement paragraph for the protocol subsection:**

> Machine readiness denotes a processed part waiting for collection at a machine output. Each machine has a readiness mask in the POSE message, with bit i set while BOX_i is waiting at that machine. Bits identify parts, not bays; for example, a mask of 5 (binary 0101) indicates that BOX_0 and BOX_2 are waiting. A zero mask means that no output is ready for collection, but does not indicate whether the machine is idle, processing, or able to accept a new part. READY messages additionally identify the output bay and the robot-center pickup pose. Readiness remains active until the supervisor registers pickup, when it clears the corresponding bit and sends CLEAR. C++ controllers normally access this information through the readiness and pickup-pose methods rather than decoding masks directly.

In the API discussion, show `machineAReady(2)`, `machineAReadyBay(2)`, and `machineAReadyPose(2, pickupPose)` together. Explicitly state that the last method returns whether a valid ready pose is available and writes it into `pickupPose`.

Evidence: [manuscript protocol](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:256), [mask construction](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:323), [processing start](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:402), [readiness clearing](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:469), [processing completion](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:551), [ready-pose getters](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/student_controller_cpp/robot_navigation.cpp:328).

## 2. Make the overlay explain what is happening and why

**Priority: high.**

The supplied screenshot confirms the current display:

```text
ORDER GRRB   mode=random   score=0   magnet=OFF
attached=none   boxes: 0:G  1:R  2:R  3:B
readyMask A=0 B=0   bay values are box index; -1=empty
A in/out: -1/-1 -1/-1   B in/out: -1/-1 -1/-1
```

The display is technically decodable, but it demands too much prior knowledge. The two pairs after each machine have no explicit bay numbers; each slash separates input from output; numbers represent boxes; `-1` is an internal sentinel; and machine A/B and bay IDs are not visibly marked on the field. The same word “ready” can be mistaken for input availability. Initial order and current part types are displayed without explaining that only the latter changes.

More consequentially, input occupancy does not tell the learner whether processing is running or blocked. Box color does not distinguish a blue part awaiting delivery from a blue part already delivered. A raw score does not explain progress through the four-box task.

A proposed normal view should show:

| Area | Proposed information | Already available to the supervisor? |
| --- | --- | --- |
| Task summary | Initial types, mode, score/max score, delivered count/4, elapsed simulation time | Yes; denominator and delivered count need simple derivation |
| Robot | Magnet reported ON/OFF; supervisor-tracked box | Yes; use labels that reflect how these are obtained |
| Each machine bay | Input box, processing/blocked state, output box ready to collect | Yes |
| Processing | Remaining time, or simply “processing” if hidden timing is a learning constraint | Yes, from the processing deadline |
| Last task event | Accepted, processed, collected, delivered, or rejected, with box and reason | Events exist; retain a short event record for the overlay |
| Controller intent | Current state, target, reason for waiting | Present in the robot process/console; requires added telemetry to show on the supervisor overlay |

For example, an illustrative machine view could read:

```text
Machine A — red → green
Bay 0   Input: BOX_1, waiting for output to clear
        Output: BOX_0, green, ready to collect
Bay 1   Input: empty
        Output: BOX_2, green, ready to collect
```

Keep raw masks and `in/out` tuples in an optional protocol/debug view. Changing decimal masks to binary alone would still leave the student decoding an implementation representation.

Add visible labels for A/B, input/output, and bay 0/1 in the scene. This matters because bay 1 is below bay 0 for Machine A, but above bay 0 for Machine B in the ENU map. Preserve 0-based IDs in the UI so logs, code, protocol, and scene agree. Use text alongside color. Give the overlay a consistent backing surface so labels remain readable across viewpoints.

The supervisor already has enough information for a much clearer machine view without changing the wire protocol. Showing controller intent is a separate change because that information currently stays inside the student controller.

Evidence: [overlay implementation](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:635), [bay definitions](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:679), [supplied screenshot](C:/Users/jabra/Repos/webots_logistics_pbl/imgs/simulation_scenario.png), [robot telemetry](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/student_controller_cpp/student_controller.cpp:111).

## 3. Separate the information the supervisor knows from the student API

**Priority: high.**

The paper proposes dynamic bay selection and scheduling, but the present protocol exposes output readiness, not input occupancy, processing status, queued inputs, or remaining processing time. These values are displayed or stored by the supervisor without being broadcast to the student API.

A student can implement scheduling by tracking their own accepted operations, but the API does not currently provide a direct `inputBayFree` or equivalent query. A zero output-ready mask cannot substitute for that query: an input may already be processing.

The manuscript should distinguish three categories: provided state, state students must maintain, and proposed protocol extensions. If observable machine-state scheduling is intended as a core exercise, a structured per-bay status message and corresponding getters would make that exercise substantially clearer. If maintaining this state is itself the assignment, say so explicitly.

Evidence: [complete protocol table](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:268), [public interface](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/student_controller_cpp/robot_navigation.hpp:19), [supervisor bay state](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:135).

## 4. Correct the state-transition explanation and include collection actions

**Priority: high; small manuscript changes.**

The controller-pattern instruction to “wait for the corresponding ready pose to be reached before entering the output bay” confuses message availability with navigation completion. Replace it with:

> After placing a part at a machine input, clear the input area and move to the output approach pose. Wait until the API provides a ready pickup pose for that part, then enter the output bay, pick the part, and reverse to the clear pose before continuing.

The task-flow figure goes directly from `READY A` to “drop at Machine B”, and from `READY B` to outgoing delivery. It omits pickup at both machine outputs. Either add those actions or explicitly label the figure as a processing-dependency diagram rather than a controller action flow. For this paper, adding collection and clearance makes the API's value more visible.

`ORDER` describes the initial types in persistent box-index order. It is not a live state query. The reference controller reads the initial type once and relies on its state machine and ready events for the subsequent process flow. Change “read state from ORDER” to “read initial type from ORDER”.

Evidence: [flow figure](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:191), [controller-pattern wording](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:549), [implemented collection sequence](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/student_controller_cpp/student_controller.cpp:310).

## 5. Describe success, scoring, and failure separately

**Priority: high for reporting and for the next project iteration.**

Each initially red, green, or blue box can earn 3, 2, or 1 points respectively. Thus the default shuffled `RRGB` task has a maximum of **9 points** for **4 deliveries**. For a manually configured order, the maximum is `3*nR + 2*nG + nB`. Machine placement earns its point on acceptance, even when the input must wait for an occupied output.

The reference C++ policy is deliberately a one-box example, with an expected successful score of 1–3 depending on BOX_0's initial type. It does not solve the complete four-box task. This is stated in the manuscript, but a short baseline/assignment distinction near the contribution would prevent the reader from missing it.

There is also a correctness limitation in the supplied example: it advances after fixed settling waits following drops, without an explicit acceptance result. If a machine drop is rejected, its ready state may never arrive and the example can wait indefinitely. After an outgoing drop, it can reach `FINISHED` without verifying that delivery was accepted. These are source-visible failure paths; their frequency was not measured in this review.

The supervisor's final-delivery test is the box center inside one outgoing rectangle. It does not enforce a particular outgoing slot, per-slot occupancy, full-footprint containment, or final orientation. Phrase the manuscript's geometric rule accordingly; “within the correct destination bay” is too broad for final delivery.

For a subsequent implementation, prioritize explicit drop-result feedback, bounded waits with diagnostic reasons, and a task-complete condition based on accepted deliveries. Until then, describe the reference controller as demonstrating the nominal route and explain its recovery limitations.

Evidence: [acceptance and scoring](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:420), [delivery predicate](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:524), [fixed wait after machine drop](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/student_controller_cpp/student_controller.cpp:287), [finish path](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/student_controller_cpp/student_controller.cpp:431).

## 6. Qualify what connector and attachment status mean

**Priority: high for technical accuracy.**

There are three different states: Webots connector locking state, the navigation wrapper's cached magnet flag, and the supervisor's inferred attached box. The supervisor chooses the nearest eligible box within a 55 mm planar distance of its nominal magnet point after receiving `MAGNET_ON`. It does not read the identity of a physical connector peer. The overlay's `magnet` value is driven by received messages.

The R2025a Connector documentation explicitly distinguishes locking state from the existence of a physical link. Presence-gated locking is the intended pickup mechanism here, but `isLocked()` and the cached `magnetIsOn()` should not be described as independent proof that a particular box is physically attached. [Webots R2025a Connector reference](https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/connector.md).

Suggested wording:

> The robot-side wrapper checks connector presence, requests locking, and reports its locking state through MAGNET_ON/OFF. The supervisor maintains a separate logical attachment estimate by associating the magnet location with the nearest eligible box. This estimate supports task bookkeeping and scoring.

Also report the deliberate manipulation simplifications: the active connector uses a 30 mm distance tolerance, an axis tolerance close to pi, no discrete rotational-alignment constraint, snapping, and unlimited tensile/shear strength. These are defensible educational choices, but readers need them to interpret navigation and manipulation results. The pose feedback is supervisor ground truth, and machine transfer is a timed teleportation; neither should be mistaken for validated physical sensing or processing.

Evidence: [pickup implementation](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/student_controller_cpp/robot_navigation.cpp:581), [logical attachment association](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:496), [connector configuration](C:/Users/jabra/Repos/webots_logistics_pbl/worlds/logistics_pbl_enu.wbt:945).

## 7. Make the educational contribution concrete

**Priority: high for the paper's positioning.**

The manuscript offers useful extension ideas, but does not yet specify an instructional activity sufficiently for another teacher to reproduce it. Add the intended course level, prerequisite programming/control knowledge, approximate activity duration, team arrangement, supplied code, required student work, deliverables, and assessment criteria. These details should come from the planned or actual teaching context, not be invented as completed classroom experience.

A compact proposed activity table could connect:

| Learning objective | Student work | Observable evidence |
| --- | --- | --- |
| Event-driven control | Extend the one-box state machine to every initial type | State diagram and traces covering R/G/B routes |
| Motion/action composition | Choose approach, service, and reverse-clear actions | Explained route choices and successful placements |
| Diagnosis and recovery | Detect rejected drops or missing pickup/ready events | An injected failure with an explained recovery trace |
| Scheduling | Handle all four boxes and justify bay/collection order | All deliveries plus comparison under matched task conditions |

A working demonstration does not establish learning effectiveness. The current conclusion appropriately places classroom evaluation in future work; make that boundary explicit earlier. A survey can assess usability and perceived learning, while claims about learning improvement need evidence tied to the learning objectives, such as assessed artifacts or pre/post tasks.

The contribution statement could be more precise:

> The contribution is an inspectable teaching scaffold that separates task rules, robot actions, and student policy within one warehouse exercise. A complete single-part example supports progression toward multi-part coordination and controller experimentation.

The related-work section should then compare against the closest competition-specific teaching environments on those dimensions, rather than relying primarily on broad simulator comparisons.

Evidence: [contribution statement](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:78), [proposed progression](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:559), [future classroom evaluation](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:566).

## 8. Distinguish available logs from measured experiment outputs

**Priority: high.**

The project-use subsection says the logs provide valid placements, deliveries, invalid drops, waiting time, transitions, score, and simulated completion time for comparison. Some are explicit console events; others require instrumentation or post-processing. Supervisor event messages are not uniformly timestamped. Detailed robot telemetry is periodic (0.35 s), and the finished message reports score without recording an exact completion timestamp or aggregated waiting time. No experiment runner, metric exporter, or results dataset was found among the reviewed project files.

Replace that sentence with:

> The implementation exposes score and task events through the supervisor and optional state and motion telemetry through the controller. These outputs support debugging. Quantitative comparisons of waiting time, completion time, and failure rates require timestamped event collection and an explicit analysis procedure.

For technical validation, the paper would benefit from a small measured table covering the blue, green, and red BOX_0 routes, both bays, an output-blocking case, and an invalid drop. Report order, seed or assigned delays, controller/version, trial count, accepted deliveries, score, completion time, and failures. Identify nominal demonstration tests separately from robustness tests. Do not report all-four-box scheduling results using the unchanged one-box controller.

Manual order mode fixes the arrangement, not the machine delays. The RNG is seeded from wall-clock time. For fair controller comparisons, use the same orders and fixed or pre-generated per-box/per-stage delays, with repeated runs as appropriate. A common seed alone may still assign different delays to boxes when policies consume random draws in different orders.

Evidence: [metric claim](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:557), [random seed](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:212), [random delays](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:318), [periodic robot output](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/student_controller_cpp/student_controller.cpp:111).

## 9. Smaller technical and documentation corrections

**Priority: medium.**

| Issue | Finding and recommended correction |
| --- | --- |
| `rotateTo` direction | It follows signed shortest-angle error and can turn either way. The API table should not imply that it is the counter-clockwise counterpart to `rotateClockwiseTo`. |
| Arc description | `moveArc` commands constant curvature and terminates on accumulated broadcast yaw. “Constant-curvature commands with yaw-based termination” is more precise than an unqualified open-loop arc. There is no geometric arc-path tracking. |
| Demo use of arcs | The API supplies arc helpers, but the current `student_controller.cpp` does not call them. README statements that the default example uses selected arcs are stale. |
| Public API table | It omits ready-mask getters, `normalizeAngle`, and the `moveCircle` alias. Either make it exhaustive or call it “Selected C++ API methods”. Give full signatures in repository documentation. |
| Startup protocol | `START` and `ORDER` are sent once after 0.5 s; `POSE` and outstanding `READY` are repeated. Qualify the runtime figure so it does not imply periodic `ORDER`. A robot controller restarted alone has no order replay/request mechanism. |
| Shared map | Student poses and marker definitions are shared, but supervisor pickup coordinates are also hard-coded in `initializeMachineBays`. The claim that named poses have a single definition is too strong. Derive both from one definition or document the duplication. |
| Marker coverage | Several marker entries are currently commented out, and poses with different headings at identical x/y produce overlapping circles. The drawn dots are not a complete depiction of every named pose or heading. |
| Final acceptance region | Use the outgoing rectangle rule described above, rather than suggesting enforced slot assignment. |
| Bay boundary ownership | Adjacent machine input rectangles overlap by 10 mm in y. `findInputBay` picks the first matching bay, so a boundary placement can be assigned to bay 0. Consider disjoint zones or document the ownership rule before teaching dynamic bay selection. |
| Coordinate diagram | Include x/y direction, heading convention, machine IDs, and bay IDs. The plotted Machine B bay-0 output y=0 is an approximation to the current y=-0.010 m; label the figure schematic if coordinates are simplified. |

Evidence: [rotation control](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/student_controller_cpp/robot_navigation.cpp:366), [arc implementation](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/student_controller_cpp/robot_navigation.cpp:538), [one-time order broadcast](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:615), [bay geometry](C:/Users/jabra/Repos/webots_logistics_pbl/controllers/logistics_supervisor_cpp/logistics_supervisor_cpp.cpp:679), [API table](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:505).

## 10. Improve figures, provenance, and references before submission

**Priority: medium, with some straightforward corrections.**

The paper contains task-flow, runtime-sequence, and component/data-flow diagrams. Keep their purposes distinct: task actions, temporal protocol behavior, and ownership boundaries. There is room to shorten implementation inventory prose and use that space for a worked box lifecycle and the educational activity/validation tables.

The simulation image is inserted at only 70% of one IEEE column and includes extensive empty background. Its embedded overlay text is too dense for that intended scale. Use a tightly framed, labeled scene image and a separate readable state/overlay detail. Verify the final compiled result at normal reading size. Add heading arrows or a clear convention to the pose diagram, since the API consumes orientation as well as position.

The README and world identify RobotAtFactory Lite inspiration; the paper frames the scenario as derived from RobotAtFactory 4.0. Explain the separate provenance of geometry, process rules, and educational simplifications. The statement that all unlisted official elements are retained is stronger than this implementation review can substantiate. An explicit retained/simplified/omitted comparison is safer and more reproducible.

The bibliography entry for the cited 2026 rulebook still says that its details need author verification and contains no direct rule-document URL. I could not verify that exact edition from the retrieved authoritative sources during this review. Confirm its edition, publication date, and rule clauses before asserting equivalence of scoring, processing, or competition procedures. The date 2025 for a 2026 rulebook may be a valid publication date; it should not be silently “corrected”.

Other edits:

- Correct “Cyberrobotics” to “Cyberbotics”, “CopeliaSim” to “CoppeliaSim”, and “Instance Navigation” to “Instantiate Navigation”.
- Check `refS01`: the cited title is a battery-modeling paper, which is an unusual source for the general list/comparison of simulators.
- Support or soften the claims about SimTwo's adoption, documentation, and language suitability; no direct evidence is attached to those comparative claims in the manuscript.
- Fix `and and` in the project repository's author list. Keep access dates in bibliographic notes rather than embedded in URL values.
- Remove unused placeholder bibliography entries when finalizing sources; they are not currently evidence and need not be replaced unless cited.
- Apply the venue's actual double-blind policy to identifying repository links and self-reference wording; an anonymous title block alone does not settle that policy question.
- Identify the released project revision used for any future experiments. This review's base commit was `1dc2fa30df0b9014ba5025f11f7a450e859200e9`, with local working-tree changes.

Evidence: [figure sizing](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:108), [competition-retention wording](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/conference_101719.tex:117), [bibliography](C:/Users/jabra/Repos/webots_logistics_pbl/CBL___Webots___RAF___ROBOT_2026/references.bib:1), [world provenance label](C:/Users/jabra/Repos/webots_logistics_pbl/worlds/logistics_pbl_enu.wbt:60).

## Recommended order of work

1. Correct readiness, initial-order semantics, collection steps, connector reporting, and acceptance rules in the manuscript and API documentation.
2. Replace the normal overlay with labeled per-bay states, task progress, and a last-event explanation; keep raw protocol values optional.
3. Add acceptance feedback and completion checks, then decide whether explicit input-bay status belongs in the student API or in the student assignment.
4. Add reproducible technical validation and document exactly which outputs were measured.
5. Specify the educational activity and assessment plan, then align figures, claims, provenance, and references with that contribution.
