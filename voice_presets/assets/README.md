# Voice preset assets

Put the four pre-recorded clips in this directory, then set their `file` fields
in `../config/presets.yaml`. Example:

```yaml
UP:
  name: xin_chao
  file: assets/xin_chao.wav
  volume_percent: 90
```

The runtime accepts `wav`, `mp3`, `ogg`, `flac`, and `m4a`; it converts the
selected file to 16 kHz mono PCM locally before sending it to the R1 speaker.
Paths outside this directory are rejected deliberately.
