# ChatGPT Review — TASK 001-D1 verdict and D2 decision

**Reviewed D1 implementation:** `d93b45ffe6239281501962edd9d3e21c70af3021`  
**Reviewed D1 evidence:** `7e51da43455f41d037ef0daa245cb538f9a57b6c`  
**Verdict:** `D1_EXPERIMENT_ACCEPTED__PROGRESSIVE_NOT_DEFAULT__SWITCH_DEFAULT_TO_LAZY_PLACE`

## 1. D1 evidence is accepted as an experiment

The progressive-preplace hypothesis is confirmed computationally:

- full tests: `128 passed in 17.44s`;
- suite P95: `4.444 s -> 3.925 s`;
- carrot P95: `4.480 s -> 3.932 s`;
- carrot coarse requested rows: `7705 -> 2839` (~63% reduction);
- gluestick coarse requested rows: `604 -> 508`;
- tennis coarse requested rows: `272 -> 224`;
- relation-found rate: 100% in both modes;
- 16-case downstream full-chain: `10/16 -> 10/16`;
- zero lazy-success/progressive-failure cases.

Therefore D1 successfully demonstrates that progressive preplace screening can substantially reduce strict coarse endpoint work.

Keep `lazy_place_progressive_preplace` in the codebase as an opt-in experimental mode.

## 2. Progressive must NOT become the production default

D1 is not strictly selection-equivalent to the already validated `lazy_place` path.

The committed evidence reports selected-relation differences for:

- tennis / `current_table`;
- carrot (`carriot`) / `current_table`.

This matters because the stated optimization contract was lossless compute scheduling. Stopping after the first clearance rank that yields a complete relation effectively introduces an additional priority over unscreened higher-clearance alternatives. Even though the current downstream success count is unchanged, the chosen relation can differ.

There is also a significant fast-case latency regression:

- tennis P95: approximately `0.205 s -> 0.820 s`;
- gluestick P95: approximately `1.709 s -> 1.833 s`.

The suite still improves because carrot dominates the tail, but the gain over C2 is not large enough to justify changing selection semantics for production now.

Do not try to hide these differences by changing ranking, seeds, tolerances, candidate sets, or benchmark cases in D2.

## 3. Production choice: validated C2 `lazy_place`

The original deployment goal was relation screening within about 5 seconds while preserving correctness.

C2 `lazy_place` already satisfies that target on the frozen smoke suite:

- suite P95 around `4.44 s` in the latest same-checkout D1 comparison;
- prior expanded D0 validation showed eager and lazy relation recall matching;
- expanded D0 full-chain: eager `10/16`, lazy `10/16`;
- zero eager-success/lazy-failure cases;
- zero selected-relation differences in D0.

Therefore the correct D2 production decision is:

```text
production relation-screen default = lazy_place
```

not progressive.

## 4. D2 must be a tiny default-only change

Create a separate D2 commit.

Requirements:

1. locate the actual production construction/configuration path for `PickPlaceCoordinator`;
2. make `lazy_place` the production default;
3. keep both `eager` and `lazy_place_progressive_preplace` selectable explicitly for benchmarks/debugging;
4. do not change any relation-screen algorithm in D2;
5. do not change seeds, tolerances, candidate grids, collision rules, MotionGen parameters, interpolation settings, or robot execution;
6. do not remove diagnostics or benchmark modes.

If the constructor default itself is the production source of truth, changing:

```python
relation_screen_mode: str = "eager"
```

to:

```python
relation_screen_mode: str = "lazy_place"
```

is acceptable, provided tests/benchmarks that need eager explicitly request it. If production has a higher-level config/CLI default, change the production source of truth instead of creating conflicting defaults.

## 5. D2 validation

After the default-only change:

1. run full repository tests;
2. run the three smoke screen-only tasks through the **production default path** (do not pass a relation-screen mode override);
3. confirm diagnostics say the active mode is `lazy_place`;
4. confirm relation-found remains 3/3;
5. record P50/P95 for the default smoke run;
6. run one repetition of the existing 16-case full-chain matrix through explicit `lazy_place` or the production-default path if the benchmark can exercise it without ambiguity;
7. verify success remains `10/16` and there is no new exception.

This D2 step is not expected to improve the existing 10/16 downstream success rate; it only promotes the validated relation-screen implementation.

Commit compact evidence to:

```text
benchmarks/task001/task001_d2_default_switch_summary.json
```

## 6. Important next task after D2

Do **not** attempt to fix the six equal baseline full-chain failures inside D2.

They are now the highest-value reliability problem for pick/place:

```text
relation screen: validated and <5 s
full chain: only 10/16 on the expanded matrix
```

After D2 is committed, stop and wait for ChatGPT review. The next task will freeze and classify the six failed cases by exact stage/reason and optimize full-chain planning success without weakening collision or terminal constraints.

## 7. Required handoff

Append to `docs/CODEX_TASK_001_D.md`:

```text
# D1 reviewer verdict / D2 completion

D1 reviewed by ChatGPT: yes
Progressive retained as experimental mode: yes/no
Production mode selected: lazy_place
D2 commit:
Production default source:
Full tests:
Default-path smoke relation success:
Default-path smoke P50/P95:
Active mode diagnostics:
16-case full-chain success:
New exceptions/regressions:
D2 summary path:
Raw logs:
Open questions for ChatGPT:
```

Stop after D2 evidence is committed. Do not start reliability changes yet.
