"""Segmenter interface: image + box prompts -> masks. SAM 2 implements it;
another segmentation model can be swapped in without touching the pipeline.
"""

import os

import numpy as np


class Segmenter:
    """Image + box prompts -> one boolean mask per box."""

    def segment_boxes(self, rgb, boxes):
        """rgb: HxWx3 uint8 RGB. boxes: Nx4 [x0, y0, x1, y1] pixels.

        Returns (masks: NxHxW bool, scores: N float).
        """
        raise NotImplementedError


def _default_workspace():
    # Walk up from this file to the workspace root, which holds models/.
    # Set VISION_MODELS_DIR to override.
    here = os.path.abspath(__file__)
    for _ in range(10):
        here = os.path.dirname(here)
        if os.path.isdir(os.path.join(here, "models", "sam2")):
            return os.path.join(here, "models")
    return os.path.expanduser("~/workspaces/sow_ws/visionNassembly/models")


class Sam2Segmenter(Segmenter):
    """SAM 2.1 image predictor with box prompts."""

    def __init__(self, checkpoint=None, config="configs/sam2.1/sam2.1_hiera_s.yaml", device="cuda"):
        import torch
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        models_dir = os.environ.get("VISION_MODELS_DIR", _default_workspace())
        checkpoint = checkpoint or os.path.join(models_dir, "sam2", "sam2.1_hiera_small.pt")
        self._torch = torch
        self.device = device
        self.predictor = SAM2ImagePredictor(build_sam2(config, checkpoint, device=device))

    def segment_boxes(self, rgb, boxes):
        torch = self._torch
        h, w = rgb.shape[:2]
        if len(boxes) == 0:
            return np.zeros((0, h, w), bool), np.zeros(0)
        with torch.inference_mode(), torch.autocast(self.device, dtype=torch.bfloat16):
            self.predictor.set_image(rgb)
            masks, scores, _ = self.predictor.predict(
                box=np.asarray(boxes, float), multimask_output=False)
        masks = np.asarray(masks)
        if masks.ndim == 4:          # N x 1 x H x W for several boxes
            masks = masks[:, 0]
        elif masks.ndim == 3 and len(boxes) == 1:
            masks = masks[:1]
        return masks.astype(bool), np.asarray(scores, float).reshape(-1)
