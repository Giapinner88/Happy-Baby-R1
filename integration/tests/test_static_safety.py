import re
import unittest
from pathlib import Path

HB_ROOT = Path(__file__).resolve().parents[2]


class StaticSafetyTest(unittest.TestCase):
    def test_integration_has_no_motor_api(self):
        forbidden = ("LowCmd", "LocoClient", "ChannelPublisher", "robot_action")
        roots = [HB_ROOT / "integration" / "src"]
        hits = []
        for root in roots:
            for path in root.rglob("*"):
                if path.suffix not in {".cpp", ".hpp", ".py"}:
                    continue
                text = path.read_text(encoding="utf-8")
                for token in forbidden:
                    if token in text:
                        hits.append(f"{path}:{token}")
        self.assertEqual([], hits)

    def test_conversation_toggle_stays_in_audio_coordinator(self):
        source = (HB_ROOT / "integration" / "src" / "main.cpp").read_text()
        self.assertIn("g_conv_mode", source)
        self.assertIn("DetectConversationToggle", source)
        self.assertIn('" conv="', source)
        self.assertIn('"conv_mode="', source)
        self.assertIn("g_conv_mode = false", source)

    def test_remote_timeout_clears_latched_hands_free_mode(self):
        source = (HB_ROOT / "integration" / "src" / "main.cpp").read_text()
        self.assertIn("void InvalidateRemoteState()", source)
        self.assertIn("g_conv_mode = false", source)
        self.assertIn("g_remote_ms = 0", source)
        self.assertIn("if (!in.remote_alive && remote_ms > 0)", source)

    def test_double_select_preempts_server_and_queued_speaker_audio(self):
        app = (HB_ROOT / "voice" / "hb_voice" / "app.py").read_text()
        speaker = (HB_ROOT / "voice" / "hb_voice" / "output.py").read_text()
        interrupt_handler = app[app.index("async def on_double_tap_interrupt"):]
        self.assertIn("await llm.send_client_event(ResponseCancelEvent())", interrupt_handler)
        self.assertNotIn("if new.voice_speaking", interrupt_handler.split("@gate.add_callback", 1)[0])
        self.assertIn("new.interrupt and not old.interrupt", speaker)
        self.assertIn("not snapshot.speaker_allowed or snapshot.interrupt", speaker)

    def test_f1_disable_fallback_restores_local_ptt_path(self):
        app = (HB_ROOT / "voice" / "hb_voice" / "app.py").read_text()
        failure_path = app[app.index("except Exception as error:", app.index("sync_conversation_mode")):]
        self.assertIn("if not requested:", failure_path)
        self.assertIn("await mic_processor.set_conversation_mode(False)", failure_path)

    def test_voice_bridge_has_no_action_mode(self):
        source = (HB_ROOT / "voice" / "unitree_bridge" / "r1_bridge.cpp").read_text()
        self.assertNotIn("LocoClient", source)
        self.assertNotIn("ActionMode", source)
        self.assertNotIn('mode == "tts"', source)
        self.assertNotIn('mode == "volume"', source)
        self.assertIn("client.SetVolume", source)

    def test_production_voice_keeps_ptt_and_adds_gated_hands_free_mode(self):
        app = (HB_ROOT / "voice" / "hb_voice" / "app.py").read_text()
        mic = (HB_ROOT / "voice" / "hb_voice" / "input.py").read_text()
        runner = (HB_ROOT / "integration" / "scripts" / "run_voice.sh").read_text()
        env = (HB_ROOT / "integration" / "config" / "stack.env.example").read_text()
        self.assertIn("SemanticTurnDetection", app)
        self.assertIn("sync_conversation_mode", app)
        self.assertIn("turn_detection=(", app)
        self.assertIn("snapshot.conv_mode", mic)
        self.assertIn("UserStoppedSpeakingFrame()", mic)
        self.assertNotIn("UNITREE_ACTIONS", app)
        self.assertIn("browser_audio=disabled", mic)
        self.assertIn('exec "$PYTHON" -m hb_voice', runner)
        self.assertNotIn("webrtc", runner.lower())
        self.assertNotIn("VOICE_RUNTIME_MODE", env)

    def test_ptt_commit_waits_for_pipeline_audio(self):
        mic = (HB_ROOT / "voice" / "hb_voice" / "input.py").read_text()
        bot = (HB_ROOT / "voice" / "hb_voice" / "app.py").read_text()
        barrier = mic.index("await self._turn_commit_barrier()")
        commit = mic.index("UserStoppedSpeakingFrame()", barrier)
        self.assertLess(barrier, commit)
        self.assertIn("await self.push_frame(buffered_frame, FrameDirection.DOWNSTREAM)", mic)
        self.assertNotIn("await self.queue_frame(buffered_frame, FrameDirection.DOWNSTREAM)", mic)
        self.assertIn("worker.flush_pipeline(timeout=3.0)", bot)
        self.assertIn('worker.cancel(reason="ptt_audio_flush_timeout")', bot)

    def test_tuning_controls_response_voice_and_volume(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        app = (HB_ROOT / "voice" / "hb_voice" / "app.py").read_text()
        config = (HB_ROOT / "voice" / "hb_voice" / "config.py").read_text()
        output = (HB_ROOT / "voice" / "hb_voice" / "output.py").read_text()
        # Pin the family, not the version: bumping the Realtime model is a
        # tuning decision and must not fail the safety suite.
        self.assertRegex(tuning, r'model: "gpt-realtime[^"]*"')
        self.assertIn('voice: "sage"', tuning)
        self.assertIn('provider: "elevenlabs"', tuning)
        self.assertIn('model: "eleven_flash_v2_5"', tuning)
        self.assertIn("ElevenLabsTTSService", app)
        self.assertIn('output_modalities = ["text"]', app)
        self.assertLess(
            app.index("pipeline_steps.append(external_tts)"),
            app.index("speaker_processor,"),
        )
        self.assertIn("gain_db: 0.0", tuning)
        self.assertIn('source: "alsa_usb"', tuning)
        self.assertIn("require_external_mic: true", tuning)
        self.assertIn("hands-free F1 must start on the PC2 external mic", config)
        self.assertIn("Double-F1 hands-free input starts on the PC2 ALSA mic", app)
        self.assertIn('vad_eagerness: "medium"', tuning)
        self.assertNotIn("nudge_after_s", tuning)
        self.assertNotIn("run_idle_nudges", app)
        self.assertNotIn("ResponseCreateEvent", app)
        self.assertIn("response_volume_percent:", tuning)
        self.assertIn('mode: "both"', tuning)
        self.assertIn("allow_during_startup: true", tuning)
        self.assertIn("str(self._response_volume_percent)", output)
        self.assertNotIn("ELEVENLABS_API_KEY=", tuning)
        self.assertNotIn("ELEVENLABS_VOICE_ID=", tuning)

    def test_voice_gesture_is_configured_but_starts_disabled(self):
        # High-level tuning is split through include: files.  The gesture
        # settings live in gestures.yaml, not in the entry-point file.
        tuning = (HB_ROOT / "controller" / "config" / "gestures.yaml").read_text()
        self.assertIn("gesture_voice_marker: /run/hb/voice_speaking", tuning)
        self.assertIn("gesture_voice_auto_start: false", tuning)
        self.assertIn("gesture_voice_speed: 1.15", tuning)
        # The clip name is a tuning choice, so check whatever is configured
        # actually resolves, in the same order Application::InitGestures does.
        name = re.search(r"^gesture_voice_name:\s*(\S+)", tuning, re.MULTILINE)
        self.assertIsNotNone(name, "gesture_voice_name is missing")
        folder = HB_ROOT / "controller" / "policies" / "gestures"
        candidates = [
            folder / f"{name.group(1)}.loop.npz",
            folder / f"{name.group(1)}.npz",
        ]
        self.assertTrue(
            any(path.is_file() for path in candidates),
            f"no gesture clip for gesture_voice_name={name.group(1)}",
        )

    def test_voice_gesture_uses_high_level_owner_and_existing_safety_gates(self):
        voice_tools = (HB_ROOT / "voice" / "hb_voice" / "gesture_tools.py").read_text()
        voice_config = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        owner = (
            HB_ROOT / "controller" / "src" / "input" / "VoiceGestureOwner.hpp"
        ).read_text()
        app = (
            HB_ROOT / "controller" / "src" / "app" / "Application.cpp"
        ).read_text()
        high_level_config = (
            HB_ROOT / "controller" / "config" / "gestures.yaml"
        ).read_text()
        high_level_service = (
            HB_ROOT / "integration" / "systemd" / "hb_high_level.service.in"
        ).read_text()
        voice_service = (
            HB_ROOT / "integration" / "systemd" / "hb_voice.service.in"
        ).read_text()

        self.assertIn('mode: "off"', voice_config)
        self.assertIn('"off", "shadow", "execute"', voice_tools)
        self.assertIn("SUPPORTED_GESTURES", voice_tools)
        self.assertIn("asyncio.open_unix_connection", voice_tools)
        self.assertIn("phản hồi ĐẦU TIÊN phải là đúng một tool call", voice_tools)
        self.assertIn("robot sẽ gửi lệnh", voice_tools)
        self.assertIn("Chờ tool trả kết quả rồi mới được nói", voice_tools)
        self.assertNotIn("LowCmd", voice_tools)
        self.assertNotIn("ChannelPublisher", owner)
        self.assertNotIn("LowCmdSender", owner)
        self.assertNotIn("LowCmd_", owner)
        self.assertIn('"/run/hb/gesture_owner.sock"', voice_config)
        self.assertIn(
            "gesture_voice_command_socket: /run/hb/gesture_owner.sock",
            high_level_config,
        )
        expected_slots = {
            "wave": 1,
            "heart": 6,
            "handshake": 2,
            "cool_pose": 7,
            "determined": 8,
            "give_gift": 4,
        }
        for action, slot in expected_slots.items():
            self.assertRegex(
                high_level_config,
                rf"gesture_voice_{action}_slot:\s*{slot}\b",
            )
        for gate in (
            "!armed_",
            "AppState::kLocomotion",
            "teleop_owns_upper_body",
            "stability_guard",
            "movement_requested",
            "stand_dwell_incomplete",
            "looping_asset_unsupported",
        ):
            self.assertIn(gate, app)
        self.assertIn("gesture_handover_guard_", app)
        self.assertIn("User=@RUN_USER@", high_level_service)
        self.assertIn("User=@RUN_USER@", voice_service)

    def test_deploy_discovers_robot_and_can_restart_only_voice(self):
        deploy = (HB_ROOT / "integration" / "scripts" / "deploy_stack.sh").read_text()
        finder = (HB_ROOT / "controller" / "scripts" / "_find_robot.sh").read_text()
        activate = (HB_ROOT / "integration" / "scripts" / "activate_services.sh").read_text()
        self.assertIn('source "$HB_ROOT/controller/scripts/_find_robot.sh"', deploy)
        self.assertIn("find_robot", deploy)
        self.assertIn("192.168.145.209", finder)
        self.assertIn("--restart-voice", deploy)
        self.assertIn("--accept-policy", deploy)
        self.assertIn('update_model_manifest.sh" --accept', deploy)
        self.assertIn("systemctl restart hb_integration.service hb_voice.service", deploy)
        self.assertIn('if [[ "$NO_RESTART" == "0" ]]', activate)

    def test_motor_owner_stops_and_restarts_only_when_disarmed(self):
        ctl = (HB_ROOT / "integration" / "scripts" / "stack_ctl.sh").read_text()
        for command in ("stop-all", "stop-high", "restart-all"):
            case = ctl.split(f"    {command})", 1)[1].split(";;", 1)[0]
            self.assertIn("refuse_armed", case, command)
            self.assertLess(case.index("refuse_armed"), case.index("sudo systemctl"), command)
        self.assertIn('grep -qx "high_armed=0"', ctl)
        self.assertIn('grep -q "^high_state=DISARMED"', ctl)
        deploy = (HB_ROOT / "integration" / "scripts" / "deploy_stack.sh").read_text()
        self.assertIn("stack_ctl.sh' $COMMAND", deploy)

    def test_voice_only_deploy_never_pulls_or_restarts_high_level(self):
        deploy = (HB_ROOT / "integration" / "scripts" / "deploy_stack.sh").read_text()
        voice_case = deploy.split("    deploy-voice)", 1)[1].split(
            "    deploy-integration)", 1
        )[0]
        self.assertIn("sync_voice", voice_case)
        self.assertIn("build_on_robot.sh' voice", voice_case)
        self.assertIn("--job-mode=ignore-dependencies restart hb_voice.service", voice_case)
        self.assertNotIn("sync_high", voice_case)
        self.assertNotIn("hb_high_level.service", voice_case)

    def test_a_pinned_robot_address_is_verified_not_trusted(self):
        """A stale ROBOT must not abort the deploy.

        On 2026-08-05 the robot moved from the 10.42.0.x hotspot to the
        192.168.1.x LAN.  `ROBOT` was pinned to the old address, so deploy died
        with "No route to host" while the robot was reachable the whole time,
        both on the LAN and over Tailscale.
        """
        finder = (HB_ROOT / "controller" / "scripts" / "_find_robot.sh").read_text()
        # Reachability decides, for the pinned address as much as for probes.
        self.assertIn("if _robot_ssh_ok \"$preferred_robot\"; then", finder)
        self.assertIn("không SSH được; vòng $scan_round sẽ quét toàn bộ danh sách", finder)
        # SSH, not ping: sshd can be down on a host that still answers ICMP.
        self.assertIn("ssh -o BatchMode=yes", finder)
        # Tailscale is the last resort and must stay last, or it would mask a
        # local link that is faster.
        self.assertIn("HB_ROBOT_TAILSCALE", finder)
        candidates = finder.split("HB_ROBOT_CANDIDATES=(", 1)[1].split(")", 1)[0]
        self.assertLess(
            candidates.index("192.168.1.33"), candidates.index("HB_ROBOT_TAILSCALE")
        )

    def test_service_install_never_rewrites_existing_secret(self):
        install = (HB_ROOT / "integration" / "scripts" / "install_services.sh").read_text()
        self.assertIn("if [[ ! -e /etc/hb/stack.env ]]", install)
        self.assertNotIn("remove_legacy_env", install)
        self.assertNotIn("/etc/hb/stack.env >", install)

    def test_preflight_requires_external_mic_on_pc2(self):
        preflight = (HB_ROOT / "integration" / "scripts" / "preflight.sh").read_text()
        service = (HB_ROOT / "integration" / "systemd" / "hb_voice.service.in").read_text()
        self.assertIn('if [[ "$MIC_SOURCE" == "alsa_usb" ]]', preflight)
        self.assertIn("arecord -l", preflight)
        self.assertIn("is not present on PC2", preflight)
        self.assertIn('[[ "$ALSA_CARD" != "APE" ]]', preflight)
        self.assertIn("SupplementaryGroups=audio", service)
        # A missing external mic must not block the stack while a standby exists.
        self.assertIn('if [[ "$MIC_FALLBACK" == "1" ]]', preflight)
        self.assertIn("voice will start on the robot mic standby", preflight)

    def test_microphone_falls_back_to_the_robot_mic_and_recovers(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        mic_input = (HB_ROOT / "voice" / "hb_voice" / "input.py").read_text()
        app = (HB_ROOT / "voice" / "hb_voice" / "app.py").read_text()
        status = (HB_ROOT / "voice" / "hb_voice" / "resilience.py").read_text()
        self.assertIn("fallback_to_robot_mic: true", tuning)
        self.assertIn('fallback_vad_eagerness: "low"', tuning)
        # Swapping source, spotting the card again, and telling the app about it.
        self.assertIn("async def _switch_source", mic_input)
        self.assertIn("/proc/asound/cards", mic_input)
        self.assertIn("_watch_for_external_mic", mic_input)
        # A receiver left on the bus keeps streaming digital silence, so bytes
        # arriving must not be mistaken for the microphone being alive.
        self.assertIn("mic_silence_s: 20", tuning)
        self.assertIn("def _note_audio_content", mic_input)
        self.assertIn("self._silent_stall", mic_input)
        # Measured on the robot: a dead mic still dithers +/-1 LSB, so testing
        # for perfect zeros would never fire.
        self.assertIn("pcm16_peak(audio) > SILENT_PEAK_CEILING", mic_input)
        self.assertIn("SILENT_PEAK_CEILING = 32", mic_input)
        # The two sources are alternatives, never a mix.
        self.assertIn("async def _ensure_single_capture", mic_input)
        self.assertEqual(2, mic_input.count("await self._ensure_single_capture()"))
        self.assertIn("set_source_change_callback(on_mic_source_changed)", app)
        # Turn detection and noise reduction have to follow the live mic.
        self.assertIn("vad_eagerness_for(live_mic_source)", app)
        self.assertIn("noise_reduction_for(live_mic_source)", app)
        self.assertIn("sync_conversation_mode(force=True)", app)
        # The swap must be visible without reading logs.
        self.assertIn("mic_source=", status)

    def test_a_turn_with_no_speech_is_never_sent(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        mic_input = (HB_ROOT / "voice" / "hb_voice" / "input.py").read_text()
        resilience = (HB_ROOT / "voice" / "hb_voice" / "resilience.py").read_text()
        self.assertIn("min_speech_peak: 500", tuning)
        # Readiness, not just the commit, is gated: an unheard turn must never
        # reach OpenAI, which would answer the empty turn with a greeting.
        self.assertIn("self.peak < self.minimum_peak", resilience)
        self.assertIn("minimum_peak=min_speech_peak", mic_input)
        self.assertIn("PTT turn dropped", mic_input)

    def test_failed_mic_recovery_backs_off_instead_of_flapping(self):
        """A present card is not a working mic.

        Observed 2026-08-05 09:38-09:40: the receiver was plugged in but its
        transmitter was still off, so the card-presence check kept returning to
        the external mic and the silence detector kept leaving it, trading the
        source every ~20s.  The robot was usable on neither.
        """
        mic_input = (HB_ROOT / "voice" / "hb_voice" / "input.py").read_text()
        self.assertIn("RECOVER_BACKOFF_MAX_S", mic_input)
        # The wait is a variable, not the raw config value, or it cannot grow.
        self.assertIn("await asyncio.sleep(self._recover_delay_s)", mic_input)
        # Returning to the card is provisional until sound actually arrives.
        self.assertIn("self._recovery_probe = True", mic_input)
        self.assertIn(
            "self._recover_delay_s = min(previous * 2, RECOVER_BACKOFF_MAX_S)", mic_input
        )
        # And a mic that really came back must not stay throttled.
        self.assertIn("self._recover_delay_s = self._fallback.recover_check_s", mic_input)

    def test_hands_free_noise_never_reaches_the_realtime_vad(self):
        """Auto mode must honour the same speech floor as push-to-talk.

        Hands-free has no button edge, so Semantic VAD alone decides when a
        turn ends.  On 2026-08-05 servo noise and the tail of the robot's own
        speaker committed five turns nobody spoke; each was answered with a
        generic greeting on top of the reply still playing, and the two
        overlapping ElevenLabs streams sounded like the voice changing.
        """
        mic_input = (HB_ROOT / "voice" / "hb_voice" / "input.py").read_text()
        # The conversation branch must run through the gate, not push raw.
        self.assertIn(
            "for gated_frame in self._hands_free_frames(frame, audio):", mic_input
        )
        self.assertIn("def _hands_free_frames", mic_input)
        self.assertIn("self._hands_free_open_until = now + HANDS_FREE_HANGOVER_S", mic_input)
        # A hangover keeps quiet syllables inside a sentence...
        self.assertIn("HANDS_FREE_HANGOVER_S = ", mic_input)
        # ...and a pre-roll keeps the soft onset of the first one.
        self.assertIn("HANDS_FREE_PREROLL_CHUNKS = ", mic_input)
        # An open hangover must not survive the robot's own reply.
        self.assertIn("self._reset_hands_free_gate()", mic_input)

    def test_speaker_tail_keeps_the_mic_shut_until_the_robot_is_quiet(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        output = (HB_ROOT / "voice" / "hb_voice" / "output.py").read_text()
        app = (HB_ROOT / "voice" / "hb_voice" / "app.py").read_text()
        self.assertIn("echo_tail_s:", tuning)
        # Tunable, because the DDS hop to PC1 is not observable from here.
        self.assertIn("self._playback_tail_s = playback_tail_s", output)
        self.assertIn("playback_tail_s=config.echo_tail_s", app)

    def test_mic_bridge_arguments_cannot_shift_and_abort_the_bridge(self):
        """`r1_bridge mic` is positional; a gap used to kill it silently.

        segment_seconds is always 0 in production, so skipping its slot pushed
        the group IP into `seconds` and the payload mode into `port`, where
        std::stoi threw out of main.  The bridge aborted with no message and the
        read loop respawned it forever - the robot was simply deaf.
        """
        mic_input = (HB_ROOT / "voice" / "hb_voice" / "input.py").read_text()
        bridge = (HB_ROOT / "voice" / "unitree_bridge" / "r1_bridge.cpp").read_text()
        self.assertIn("if self._segment_seconds > 0 or self._mic_group_ip:", mic_input)
        # No unchecked conversion may reach main again.
        self.assertNotIn("std::stoi(argv", bridge)
        self.assertIn("bool ParseInt(", bridge)
        self.assertIn(
            "inet_pton(AF_INET, group_ip.c_str(), &mreq.imr_multiaddr) != 1", bridge
        )
        # An unknown payload mode must fail instead of reading RTP as raw.
        self.assertIn("bool ParseMicPayloadMode(", bridge)

    def test_discarded_turn_audio_is_retracted_from_the_realtime_buffer(self):
        """Aborting a turn locally does not unsend what OpenAI already holds.

        A PTT turn that had started streaming and was then cut (robot began
        speaking under a held Select, or F1 switched to hands-free) left its
        audio in the server-side input buffer, where the next commit - or
        Semantic VAD, once armed - picked it up as part of somebody else's turn.
        """
        mic_input = (HB_ROOT / "voice" / "hb_voice" / "input.py").read_text()
        app = (HB_ROOT / "voice" / "hb_voice" / "app.py").read_text()
        self.assertIn("async def _discard_active_turn", mic_input)
        self.assertIn("mic_processor.set_turn_discard_hook(", app)
        hook = app[app.index("async def drop_uncommitted_turn_audio"):]
        self.assertIn(
            "await llm.send_client_event(InputAudioBufferClearEvent())",
            hook[: hook.index("mic_processor.set_turn_discard_hook(")],
        )
        # Every gate-driven teardown has to go through the helper: a bare abort
        # is what left the fragment behind.  The two survivors are the helper
        # itself and the pipeline Cancel/End path, where the session is dying.
        self.assertEqual(2, mic_input.count("self._turn_audio.abort()"))
        # Discarding must never commit: that is what UserStoppedSpeakingFrame
        # does, and it would make the robot answer half a sentence.
        discard = mic_input[mic_input.index("async def _discard_active_turn"):]
        self.assertNotIn(
            "UserStoppedSpeakingFrame()",
            discard[: discard.index("async def set_conversation_mode")],
        )

    def test_microphone_source_is_chosen_by_the_operator_not_by_health(self):
        """F2 owns the microphone source; nothing else may move it.

        Auto switching traded the source three times in three minutes on
        2026-08-06 (dead wearable transmitter, receiver still on the bus) and
        the robot was usable on neither mic.  Manual is now the default.
        """
        main = (HB_ROOT / "integration" / "src" / "main.cpp").read_text()
        gate = (HB_ROOT / "voice" / "hb_voice" / "gate.py").read_text()
        mic_input = (HB_ROOT / "voice" / "hb_voice" / "input.py").read_text()
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()

        # F2 single press, carried to voice as a sticky selection.
        self.assertIn("void DetectMicSourceToggle(bool f2_now)", main)
        self.assertIn("components.F2", main)
        self.assertIn('" micext="', main)
        self.assertIn("mic_external=", main)
        # Losing the remote must never move the microphone: unlike ptt/conv this
        # is a choice, not a live input.
        invalidate = main[main.index("void InvalidateRemoteState()"):]
        self.assertNotIn("g_mic_external =", invalidate[: invalidate.index("\n}")])
        # Nor may a stale coordinator reset it.
        self.assertIn('mic_external=flag("micext", True)', gate)
        self.assertIn("mic_external=self._snapshot.mic_external", gate)
        # Manual is the shipped default, and health paths respect it.
        self.assertIn("mic_switch: manual", tuning)
        self.assertIn("async def request_source", mic_input)
        self.assertIn("self._auto_switch = switch_mode ==", mic_input)
        self.assertIn("if not self._auto_switch or not self._fallback:", mic_input)

    def test_startup_grace_covers_hands_free_as_well_as_push_to_talk(self):
        gate = (HB_ROOT / "voice" / "hb_voice" / "gate.py").read_text()
        self.assertIn(
            "startup_allowed and (raw.conv_mode or (raw.ptt and not raw.rearm))", gate
        )

    def test_voice_speaking_is_never_released_without_an_owner(self):
        """The idle task holds its slot until after the flag is really down."""
        output = (HB_ROOT / "voice" / "hb_voice" / "output.py").read_text()
        self.assertIn(
            "if self._playback_deadline + self._playback_tail_s <= time.monotonic():",
            output,
        )
        self.assertIn("finally:\n            self._voice_idle_task = None", output)
        # The slot is claimed before the awaits, so two writers cannot each
        # spawn a task and have the loser clear the winner's flag.
        spawn = output.index("self._voice_idle_task = self.create_task(")
        self.assertLess(spawn, output.index("await self._gate.set_voice_speaking(True)"))

    def test_mode_cue_tells_the_operator_which_mode_is_live(self):
        tuning = (HB_ROOT / "voice" / "config" / "tuning.yaml").read_text()
        app = (HB_ROOT / "voice" / "hb_voice" / "app.py").read_text()
        output = (HB_ROOT / "voice" / "hb_voice" / "output.py").read_text()
        self.assertIn("mode_cue: true", tuning)
        self.assertIn("def mode_cue_pcm", output)
        self.assertIn("async def play_mode_cue", output)
        # Locally generated tones only: no cloud TTS on a mode change. Rising
        # do-mi-sol = hands-free on, falling = push-to-talk.
        self.assertIn("triad = (523.25, 659.25, 783.99)", output)
        self.assertIn("triad = tuple(reversed(triad))", output)
        # Never fight high-level for the speaker.
        self.assertIn("speaker is preempted", output)
        self.assertIn("play_mode_cue(applied and new.conv_mode)", app)


if __name__ == "__main__":
    unittest.main()
