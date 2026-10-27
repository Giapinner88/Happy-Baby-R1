Pokémon Plus GOC — no spin, no backward lean

Snapshot model_14500.pt from the 2026-10-27 GOC run (target 15000 iterations; training in progress at packaging time). 12288 environments, seed 42, foot friction 0.8–1.5.

Exact training motion pokemon_plus.npz: 3414 frames at 50 Hz (68.28 seconds). Matching pokemon.mp3 retained unchanged from the source package, with its existing timing. Deployment metadata copied verbatim from this same run framework ONNX. Actor weights and observation normalization verified against checkpoint 14500, including normalizer epsilon 0.01.

Installed inactive in Happy-Baby-R1-develop/controller/policies/dance. Hardware acceptance pending.
