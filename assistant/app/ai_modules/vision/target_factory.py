"""Shared tracker construction from color config or hash-bound grounding seed."""
import json
from pathlib import Path
import cv2
from ai_modules.vision.color_target import RedTarget
from ai_modules.vision.dac_depth import sha256
from ai_modules.vision.openai_grounding import validate_proposal, pixel_box


def tracker_sources(config, root):
    if config.get('backend', 'red') == 'red':
        return []
    if config['backend'] != 'features':
        raise ValueError('unknown tracker backend')
    seed = json.loads((Path(root)/config['seed_manifest']).read_text())
    return [Path(root)/config['seed_manifest'], Path(root)/seed['image']]


def make_tracker(config, root):
    if config.get('backend', 'red') == 'red':
        return RedTarget(config)
    from ai_modules.vision.feature_target import FeatureTarget
    sources = tracker_sources(config, root)
    seed = json.loads(sources[0].read_text())
    if sha256(sources[1]) != seed['image_sha256']:
        raise ValueError('grounding seed image hash mismatch')
    proposal = validate_proposal(seed['proposal'])
    if proposal['decision'] != 'selected':
        raise ValueError('cannot track ask/not_found proposal')
    image = cv2.imread(str(sources[1]))
    if image is None or [image.shape[1], image.shape[0]] != seed['image_size_wh']:
        raise ValueError('grounding seed image dimensions mismatch')
    box = pixel_box(proposal['bbox'], image.shape[1], image.shape[0])['xyxy']
    return FeatureTarget(config, image, box)
