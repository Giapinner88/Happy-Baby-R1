import unittest
from pathlib import Path

HB_ROOT = Path(__file__).resolve().parents[2]


class StaticSafetyTest(unittest.TestCase):
    def test_integration_has_no_motor_api(self):
        forbidden = ("LowCmd", "LocoClient", "ChannelPublisher", "robot_action")
        roots = [HB_ROOT / "r1_integration" / "src"]
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

    def test_voice_bridge_has_no_action_mode(self):
        source = (HB_ROOT / "voice_r1" / "unitree_bridge" / "r1_bridge.cpp").read_text()
        self.assertNotIn("LocoClient", source)
        self.assertNotIn("ActionMode", source)
        self.assertNotIn('mode == "tts"', source)
        self.assertNotIn('mode == "volume"', source)
        self.assertIn("client.SetVolume", source)

    def test_production_voice_is_fixed_headless_ptt(self):
        app = (HB_ROOT / "voice_r1" / "hb_voice" / "app.py").read_text()
        mic = (HB_ROOT / "voice_r1" / "hb_voice" / "input.py").read_text()
        runner = (HB_ROOT / "r1_integration" / "scripts" / "run_voice.sh").read_text()
        env = (HB_ROOT / "r1_integration" / "config" / "stack.env.example").read_text()
        self.assertIn("turn_detection=False", app)
        self.assertNotIn("UNITREE_ACTIONS", app)
        self.assertIn("browser_audio=disabled", mic)
        self.assertIn('exec "$PYTHON" -m hb_voice', runner)
        self.assertNotIn("webrtc", runner.lower())
        self.assertNotIn("VOICE_RUNTIME_MODE", env)

    def test_ptt_commit_waits_for_pipeline_audio(self):
        mic = (HB_ROOT / "voice_r1" / "hb_voice" / "input.py").read_text()
        bot = (HB_ROOT / "voice_r1" / "hb_voice" / "app.py").read_text()
        barrier = mic.index("await self._turn_commit_barrier()")
        commit = mic.index("UserStoppedSpeakingFrame()", barrier)
        self.assertLess(barrier, commit)
        self.assertIn(
            "await self.push_frame(buffered_frame, FrameDirection.DOWNSTREAM)", mic
        )
        self.assertNotIn(
            "await self.queue_frame(buffered_frame, FrameDirection.DOWNSTREAM)", mic
        )
        self.assertIn("worker.flush_pipeline(timeout=3.0)", bot)
        self.assertIn('worker.cancel(reason="ptt_audio_flush_timeout")', bot)

    def test_tuning_controls_response_voice_and_volume(self):
        tuning = (HB_ROOT / "voice_r1" / "config" / "tuning.yaml").read_text()
        output = (HB_ROOT / "voice_r1" / "hb_voice" / "output.py").read_text()
        self.assertIn('model: "gpt-realtime-1.5"', tuning)
        self.assertIn('voice: "marin"', tuning)
        self.assertIn("gain_db: 6.0", tuning)
        self.assertIn("response_volume_percent:", tuning)
        self.assertIn('mode: "both"', tuning)
        self.assertIn("allow_during_startup: true", tuning)
        self.assertIn("str(self._response_volume_percent)", output)

    def test_deploy_discovers_robot_and_can_restart_only_voice(self):
        deploy = (
            HB_ROOT / "r1_integration" / "scripts" / "deploy_stack.sh"
        ).read_text()
        activate = (
            HB_ROOT / "r1_integration" / "scripts" / "activate_services.sh"
        ).read_text()
        self.assertIn('source "$HB_ROOT/high_level_2/scripts/_find_robot.sh"', deploy)
        self.assertIn("find_robot", deploy)
        self.assertIn("--restart-voice", deploy)
        self.assertIn("--accept-policy", deploy)
        self.assertIn('update_model_manifest.sh" --accept', deploy)
        self.assertIn(
            "systemctl restart hb_integration.service hb_voice.service", deploy
        )
        self.assertIn('if [[ "$NO_RESTART" == "0" ]]', activate)

    def test_service_install_never_rewrites_existing_secret(self):
        install = (
            HB_ROOT / "r1_integration" / "scripts" / "install_services.sh"
        ).read_text()
        self.assertIn('if [[ ! -e /etc/hb/stack.env ]]', install)
        self.assertNotIn("remove_legacy_env", install)
        self.assertNotIn("/etc/hb/stack.env >", install)


if __name__ == "__main__":
    unittest.main()
