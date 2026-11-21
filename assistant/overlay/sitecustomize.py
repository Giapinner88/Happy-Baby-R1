"""Opt-in loader; only the assistant voice launcher adds this to PYTHONPATH."""
import os

if os.environ.get("R1_ASSISTANT_VOICE") == "1":
    # This switch belongs to the voice interpreter only. Camera probes use
    # system Python and must not import the voice/Pipecat extension again.
    os.environ.pop("R1_ASSISTANT_VOICE", None)
    try:
        from r1_assistant.voice import install
        install()
    except BaseException as exc:
        # Python normally ignores sitecustomize errors; fail instead of bypassing arbitration.
        import sys
        sys.stderr.write("R1 assistant voice integration failed: %s\n" % type(exc).__name__)
        os._exit(78)
