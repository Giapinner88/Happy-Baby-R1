# T003 legacy combined replay

`t003_20260811T015811Z`, `t003_20260811T020000Z`, and
`t003_20260811T020244Z` predate the T003 split. They combine nominal motion and
three injected safety transitions in one execution, so their maxima cannot
answer the nominal-tracking and safety-transition questions independently.

The raw records remain immutable and visible in
[`metadata/evidence_catalog.json`](metadata/evidence_catalog.json), but are **not selected
for either new T003 evaluation part**. Do not create additional runs with the
`t003_` prefix. Use T003-A or T003-B below.

The legacy combined result remains useful as a diagnosis: nominal portions
tracked closely, while a deadman freeze produced an observed acceleration spike.
That observation motivated the split; it is not a baseline comparison.
