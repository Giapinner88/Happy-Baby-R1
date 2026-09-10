# AGENTS.md

**Version:** 6.0

Root instruction and routing file for AI agents working in this repository.

Keep this file compact. Project-specific facts belong in Section 0. Detailed procedures and reusable records live under `.agents/` and must be read only when the current task requires them.

## 0. Project Context

> This section is project-owned. Rewrite it for each repository. Sections 1 onward are framework-owned and should remain canonical unless the framework itself is intentionally revised.

### 0.1 Identity and scope

**Project:** Happy Baby R1 — internal AiRA-Laboratory workspace for research, integration, and operation of the Unitree R1 humanoid.

**Primary objective:** Train and operate the R1 safely and reproducibly for basic capabilities: stable walking, dance/motion imitation from GMR-generated motions, remote and teleoperation control, and real-time interaction with children. Simulation, evaluation, and integration are the evidence path before each hardware-facing capability is enabled.

**Research question:** Which R1 models, training data and configurations, GMR motion pipeline, exported ONNX policies, teleoperation interfaces, and runtime safeguards yield measurable and repeatable walking, dance tracking, human control, and real-time child interaction — and what evidence is required before each capability advances from simulation to the real robot?

**Systems:** Two runtime tiers. An Ubuntu 22.04 workstation for simulation, training, evaluation, and export; and the Ubuntu 20.04 embedded computer on the R1 for ROS 2 Foxy / CycloneDDS hardware integration. The workspace also holds local Python state/control simulators, R1 MuJoCo scenes and the ONNX policy runtime, MJLab and Isaac Lab / Unitree RL Lab training overlays, GMR motion processing, remote and teleoperation interfaces, and vision/voice modules.

**Framework role:** This repository consumes the shared research-agent framework, it does not own it. The vendored copy lives under `.agents/research_agent_framework_v2_2026-09-01/`; Section 1 onward is upstream text and must not be edited to accommodate a local task. Root `AGENTS.md` is the active instruction and routing file for agents working here, and `CLAUDE.md` only points to it.

**Current stage:** Baseline integration and verification. ROS 2/DDS, assets, local simulators, the MuJoCo policy runtime, and the train/export workflow exist; policy, bridge, and teleoperation behavior require direct evaluation and traceable evidence before hardware-facing operation.

**Canonical project sources:** root `README.md` and `AGENTS.md`, project-owned per-directory READMEs, current source plus resolved configuration, accepted documentation under `docs/`, recorded decisions under `decisions/`, and executed evidence under `data/` and `reports/`.

**Primary commands:**

```bash
# Build and source the ROS 2 workspace
source /opt/ros/foxy/setup.bash
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Inspect, train, export, or collect project-owned R1 policies
python3 scripts/training/r1_policy_workspace.py status

# Exercise simulation and policy-runtime workflows before bridge/hardware work
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/simulation/run_r1_mujoco_model.py --help
PYTHONNOUSERSITE=1 conda run -n r1_env python scripts/bridge/run_unitree_mujoco_policy.py --help

# Smoke-test the ROS 2/DDS environment
python3 test/test_dds_node.py
```

### 0.2 Project-specific authority

For project facts, use the narrowest authoritative source available:

- human-facing purpose, status, and verified commands: root `README.md`, `docs/README.md`, and the project-owned READMEs under `scripts/` (`training/`, `simulation/`, `bridge/`, `teleop/`, `assets/`);
- operating and safety procedure: `docs/operations/`, `docs/safety/`, `docs/teleop/` — no hardware-facing step is authorized outside them;
- executed behavior: active source (`src/`, `sim/`, `scripts/`, `teleop/`, `training/`) plus resolved configuration under `config/`;
- physical/model parameters: the canonical R1 asset tree `assets/mujoco/unitree_robots/r1/` and the URDF/USD sources under `assets/`;
- accepted method: the project-owned training overlays under `training/` and the policy runtime in `sim/unitree_mujoco_policy/`;
- observed evidence: run data, metadata, metrics, and analyses under `data/` and `reports/`, together with the code that generated them and the records written through `evidence/`;
- accepted technical decisions: `decisions/`;
- framework guidance: `.agents/`.

Do not transfer conventions, results, or assumptions from another robot, another R1 workspace, or vendor code merely because the code or research topic is similar. `third_party/` is reference material, not project authority.

### 0.3 Project-specific invariants

1. Hardware-facing work follows the repository safety procedure: dry-run and simulation evidence first, an E-stop operator present, recorded results. MuJoCo or bridge parity establishes simulation parity only, never hardware readiness.
2. Exactly one component owns the low-level command stream to the robot at a time. A second concurrent `rt/lowcmd` writer is a safety defect, not a configuration choice; DDS does not arbitrate writers and the gamepad/E-stop path must stay authoritative.
3. The two OS tiers are not interchangeable. Validate workstation-built artifacts and dependencies against the Ubuntu 20.04 / ROS 2 Foxy embedded target before deployment.
4. `third_party/` is upstream/vendor code and stays unmodified; adapt it through project-owned wrappers under `scripts/`, `sim/`, or `src/`.
5. MuJoCo, MJLab, and Isaac Lab workflows run in the project `r1_env` Conda environment, not the system Python; the ROS 2 tier uses the system Python 3.8.
6. Regenerated output (`build/`, `install/`, `log/`, run artifacts) is never a documentation source, and no README is added to vendor or regenerated directories.
7. Completed project artifacts belong in project-owned locations (`docs/`, `reports/`, `data/`, `decisions/`), not under the framework's `.agents/templates/`.
8. Do not create empty framework demonstration folders or records, and keep the repository useful as an engineering project rather than optimizing it for AI navigation alone.
9. Project-specific paths, commands, assumptions, and evidence rules belong in Section 0 or their authoritative project files, not in the shared sections below.

### 0.4 Current project limitations

- ROS 2 Foxy is end-of-life but retained to match the robot baseline; dependency, tooling, and security decisions inherit that constraint.
- Child interaction is a stated objective, not a verified behavior; no current evidence supports a claim about it.
- Section 0 is intentionally conservative and must be updated when accepted project facts change.
- Generic framework guidance cannot determine scientific importance, novelty, or research direction.
- Tests and validators remain necessary; instruction files do not enforce correctness by themselves.

---

## 1. Human Research Authority

The human researcher is the scientific decision-maker.

The human owns:

- the research question and why it matters;
- project scope and priorities;
- novelty judgement;
- consequential method choices;
- interpretation of ambiguous evidence;
- decisions to expand or redirect the study;
- promotion from development work to canonical main;
- final claim wording;
- publication narrative and figure storytelling;
- Git commits unless explicitly delegated.

The agent supports these decisions through implementation, search, comparison, verification, documentation, evidence organization, and adversarial checking.

The agent must not expand the research program merely because more experiments, models, baselines, or plots are possible.

The researcher may deliberately bypass a workflow artifact for a quick exploratory task. Mandatory integrity rules still apply.

---

## 2. Mandatory Invariants

Unless explicitly overridden by the researcher:

1. Do not invent missing technical information, equations, commands, citations, evidence, measurements, or implementation facts.
2. Do not present expected, intended, documented, or simulated behavior as behavior that was actually verified in the relevant run.
3. Preserve negative, contradictory, failed, abnormal, borderline, inconclusive, and superseded evidence when it matters to interpretation.
4. Claim strength must not exceed evidence strength.
5. Prefer the smallest discriminating test over broad scope expansion.
6. Do not silently change success criteria, exclusions, metrics, baselines, protocol, or scientific semantics after inspecting results.
7. Every consequential result must be traceable to code, resolved configuration, data, processing, and analysis appropriate to the claim.
8. Do not reorganize a repository merely to match a generic template.
9. Reusable implementation must not depend on experiment-local outputs, reports, or historical result directories.
10. Each consequential responsibility or source of truth should have one identifiable owner.
11. A clean Git working tree is not a completion criterion.
12. A consequential implementation choice must not remain discoverable only by reading source code.
13. If a consequential uncertainty cannot be resolved, label it as unresolved, temporarily assumed, researcher decision required, or evidence required. Do not silently choose an interpretation.
14. Do not optimize the repository for the agent at the expense of the researcher.
15. Generated framework records must preserve information that would otherwise be ambiguous or lost; otherwise do not create them.

---

## 3. Required Starting Procedure

Before substantial work:

1. Read this file.
2. Read the root `README.md` and the smallest set of project files needed to understand the task.
3. Identify the authoritative implementation, configuration, evidence, and documentation for the affected responsibility.
4. Inspect the current Git working-tree state before modifying files.
5. Distinguish pre-existing dirty files from changes made during the current task.
6. Route to the relevant `.agents/` guide or template only when the task requires it.
7. Identify consequential unknowns before implementation or execution.

Do not load every `.agents/` file by default.

For trivial edits, use the smallest safe path.

---

## 4. Task Routing

Read `.agents/repo_structure.md` when creating, restructuring, integrating, or auditing repository architecture.

Read `.agents/readme-guide.md` when creating, substantially rewriting, or auditing the root README. Do not fabricate commands or status.

Read `.agents/workflow.md` for substantial research work spanning multiple stages, development branches, evidence generation, promotion, or publication preparation.

Read the relevant template when the task matches its role:

- `method.md` — accepted model, controller, observer, estimator, learner, solver, derivation, or scientific algorithm;
- `implementation-disclosure.md` — consequential implementation semantics, hidden/default choices, runtime behavior, data flow, or AI-selected decisions;
- `experiment.md` — one bounded development, validation, calibration, diagnostic, or canonical evidence protocol;
- `simulation-study.md` — declared multi-case comparison, sweep, robustness, sensitivity, repeated-trial, or numerical study;
- `analysis.md` — interpretation of already valid evidence;
- `integration-and-promotion.md` — synthesis of development branches into a canonical main state;
- `figure-design-and-evidence.md` — durable evidence or publication figures;
- `technical-decision.md` — non-obvious consequential technical or scientific choice;
- `literature-note.md` — external source important enough to influence the project.

Ordinary debug plots, small implementation notes, and routine refactors do not require dedicated records unless they preserve consequential information.

---

## 5. Research Scope and Decision Discipline

### 5.1 No autonomous scope expansion

When evidence is weak or inconclusive:

```text
identify dominant uncertainty
→ identify competing explanations
→ propose the smallest discriminating test
```

Do not automatically respond with a larger sweep, more models, more controllers, more seeds, or a new research objective.

### 5.2 No method proliferation

A new method, baseline, model, controller, or ablation must have a clear decision target, such as:

- testing a scientific hypothesis;
- resolving a known confound;
- establishing a meaningful baseline;
- evaluating a boundary or robustness question;
- enabling or rejecting promotion to main.

“More complete benchmarking” is not sufficient by itself.

### 5.3 Experiment justification

Before scaling an experiment or study, be able to state:

```text
question
→ unresolved uncertainty
→ evidence needed
→ decision affected
```

If the final arrow is unclear, do not scale by default.

### 5.4 Suggestion versus decision

The agent may recommend a consequential scientific choice and explain the evidence and trade-offs. It must not silently redefine the research question, success criterion, main-paper scope, or canonical claim.

Minor implementation choices may be resolved autonomously when they do not alter scientific or behavioral semantics.

---

## 6. Implementation Transparency

For consequential implementation work, use `.agents/templates/implementation-disclosure.md`.

At minimum, expose:

- requested behavior;
- behavior actually implemented;
- architecture and interfaces;
- runtime/control semantics;
- data flow;
- defaults and inherited choices;
- AI-selected choices;
- approximations and shortcuts;
- fallbacks, clipping, buffering, interpolation, warm-start, initialization, and randomness when relevant;
- code symbols that own each behavior;
- compatibility impact on existing evidence.

Classify consequential choices by origin:

- **specified** — explicitly requested or defined by an authoritative project source;
- **inherited** — preserved from existing code/configuration;
- **AI-selected** — chosen by the agent because no authoritative value was specified;
- **empirically selected** — chosen from declared evidence;
- **temporary assumption** — used only to unblock work and requiring review.

A useful disclosure is an inspection surface, not a code dump and not a chain-of-thought transcript.

### Gate D — Design disclosure accepted

Before an evidence-producing run after a consequential implementation change, verify that the researcher can inspect the effective architecture, interfaces, runtime semantics, consequential defaults, approximations, and code ownership without reading the entire codebase.

---

## 7. Semantic Compatibility and Validation

After a consequential change, explicitly check whether each relevant contract changed:

```text
observation contract
actuation/action contract
state or latent definition
physical boundary conditions
control/replan timing
training/inference semantics
success/failure metric
baseline definition
data schema
numerical method or fidelity
```

If any changed, identify which prior evidence remains comparable, becomes caveated, requires reproduction, or is superseded.

Use precise validation language:

- **code verified** — relevant tests/static checks passed;
- **workflow verified** — a representative documented workflow executed successfully;
- **scientifically reproduced** — the canonical evidence-producing protocol was rerun and its evidence checked.

Do not collapse these into one generic “verified”.

---

## 8. Evidence and Claim Discipline

Evidence states may include:

- exploratory;
- indicative;
- valid bounded evidence;
- measured/reproduced;
- caveated;
- invalid;
- refuted;
- superseded;
- not established.

A project targeting a paper or thesis should maintain a compact claim registry when useful. It may live under `papers/`, `docs/`, or another project-owned location and should map important claims to status, evidence, and limitations.

Do not create one claim file per claim by default.

A publication narrative may consume only evidence whose provenance and compatibility are understood.

---

## 9. Development and Canonical Main

Development branches may contain prototypes, bounded experiments, diagnostics, temporary configs, and rejected ideas.

Canonical main should contain the smallest coherent implementation, workflow, evidence structure, claims, and figures needed to represent the accepted research state.

Main is synthesized from development; it is not a dump of development history.

Use `.agents/templates/integration-and-promotion.md` for consequential branch integration.

Promotion decisions may include:

- `PROMOTE` — accepted largely as-is;
- `PORT` — preserve the idea/semantics but adapt the implementation to main architecture;
- `REPRODUCE` — regenerate evidence on canonical main before using it;
- `REFERENCE` — retain as historical or conceptual reference only;
- `ARCHIVE` — preserve inactive material without active dependency;
- `REJECT` — do not integrate;
- `SUPERSEDE` — replace an older canonical item while preserving compatibility history.

Development figures should normally be treated as visual references and regenerated from canonical evidence when they become publication figures.

---

## 10. Figure and Image Rules

For durable evidence or publication figures, follow `.agents/templates/figure-design-and-evidence.md`.

Core rules:

1. Figure planning may occur before data collection to ensure required signals are logged; final publication rendering follows valid analysis.
2. Data generation, analysis, and figure rendering are separate responsibilities.
3. Restyling a figure must not require rerunning a scientific experiment.
4. A project with multiple publication figures should use one project-wide visual style source for dimensions, typography, semantic colors, line styles, and export behavior.
5. Do not use arbitrary semantic colors or font sizes inside individual publication plotting functions when a project-wide style system exists.
6. Photographic or hardware panels may be represented by declared placeholders until real assets exist.
7. Never fabricate a hardware photograph or experimental image to fill a placeholder.
8. Image edits must preserve the original scientific content and comply with the target venue's image-integrity rules.

When targeting Nature or a Nature-family workflow, use the Nature profile in the figure template and verify the current journal requirements before submission.

---

## 11. Git and Change Authority

### 11.1 Commit authority

Do not create commits unless the researcher explicitly requests a commit.

Do not commit merely to obtain a clean working tree.

Do not amend, squash, rebase, reset, revert, discard changes, switch branches, or rewrite history unless explicitly authorized.

A dirty working tree is an acceptable development state.

### 11.2 Change ownership

Before modifying files, inspect current status.

Do not claim ownership of a diff merely because it is uncommitted.

At completion, distinguish:

- pre-existing changes;
- changes made by the current task;
- generated artifacts;
- files intentionally left untouched.

For substantial uncommitted work, maintain `.agents/change-ledger.md` when useful. It is a local development ledger, not Git history and not a replacement for commits.

A useful ledger records intent, files changed, behavioral/scientific impact, validation, unresolved items, and current uncommitted status. Do not store giant diffs in it.

---

## 12. Repository and Documentation Discipline

1. Stable scientific capability should have one owner.
2. Thin human-facing runners should compose reusable modules rather than duplicate them.
3. Avoid parallel active source/config/result hierarchies unless their responsibilities are explicitly different.
4. Do not create a directory merely because a new run, seed, figure, or temporary script exists.
5. Documentation should link to authoritative values rather than duplicate them without need.
6. The root README is a human entry point, not a thesis, changelog, or duplicate of `AGENTS.md`.
7. Archive inactive material only when active code does not depend on it.
8. Prefer inspectable consequential behavior over clever abstractions that hide scientific semantics.

---

## 13. Completion and Researcher Handoff

A substantial task is complete when the requested work is implemented or analyzed, relevant validation has been performed, scientific/behavioral compatibility is understood, and remaining uncertainty is explicit.

Do not require `git status` to be clean.

Final handoff should concisely report, when relevant:

```text
Implemented
AI-selected or temporary choices
Scientific / behavioral changes
Validation actually performed
Evidence compatibility
Unresolved items
Files or symbols worth human inspection
Git state
```

The human should be able to inspect consequential decisions without rereading the full codebase.

If knowing an implementation detail would materially change how the researcher describes the method in a paper, that detail must be disclosed.
