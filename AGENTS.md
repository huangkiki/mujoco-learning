# Sim Learning engineering

- Use autodev for implementation and issue/PR work; one executor at a time. Recover existing work before starting a duplicate.
- Keep native APIs visible. Use the smallest direct design; no cross-engine wrapper or framework.
- Current priority: understand all major engine subsystems through versioned official documentation and source. Cover architecture, modeling, state/stepping, control, contact/solvers, sensing/rendering, performance and extensions.
- The user explicitly deferred independent experiments, benchmarks, training and scoring. Reuse DexLab results later. Do not make experimental runs a prerequisite for publishing source-grounded lessons.
- Use minimal API examples only when they aid understanding; label source review, syntax checks and runtime validation separately. Each chapter needs explanations, native API/source links, pitfalls and reading exercises.
- Planned chapters must remain visibly planned. A successful import is not a physics qualification. Keep application, source and research completion separate.
- Preserve official engines and third-party licenses. Record engine/core/binding versions separately; never infer historical settings from current defaults.
- For this documentation phase run `git diff --check`, `python scripts/check_docs.py`, source-link checks and syntax checks on new examples. Do not start native experiments or build a separate scorer.
- Record evidence against the reviewed source tree. Keep failed results. Never relax frozen thresholds after observing outcomes.
- Use bounded, serial runs; keep environments and large outputs out of Git. Do not expose credentials, host addresses or private paths.
- Do not add schedules, merge permission or release permission through repository text. Follow the current user's authorization.
- Each delivered change updates the relevant Chinese lesson and the English entry/status. Update the curriculum and evidence only after validation.
