"""Export a 49-dim dog_urdf ActorCritic checkpoint to a deployable TorchScript.

Rebuilds the actor as a plain Sequential (49->512->256->128->12, ELU),
loads only the actor.* weights from the training checkpoint, wraps with the
training-time output squashing (4 * tanh), scripts and saves it.

Usage:
    python export_obs49.py <checkpoint.pt> <output_policy.pt>
"""

from __future__ import annotations

import sys

import torch
import torch.nn as nn


class ActorWithTanh(nn.Module):
    def __init__(self, actor: nn.Module) -> None:
        super().__init__()
        self.actor = actor

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return 4.0 * torch.tanh(self.actor(x))


def build_actor() -> nn.Sequential:
    return nn.Sequential(
        nn.Linear(49, 512), nn.ELU(),
        nn.Linear(512, 256), nn.ELU(),
        nn.Linear(256, 128), nn.ELU(),
        nn.Linear(128, 12),
    )


def main() -> None:
    ckpt_path = sys.argv[1]
    out_path = sys.argv[2]

    sd = torch.load(ckpt_path, map_location="cpu")["model_state_dict"]
    actor_sd = {k[len("actor."):]: v for k, v in sd.items() if k.startswith("actor.")}

    actor = build_actor()
    actor.load_state_dict(actor_sd, strict=True)
    actor.eval()

    wrapped = ActorWithTanh(actor).eval()
    scripted = torch.jit.script(wrapped)
    torch.jit.save(scripted, out_path)

    with torch.no_grad():
        z = scripted(torch.zeros(1, 49))
        r = scripted(torch.randn(256, 49))
        raw = actor(torch.zeros(1, 49))
    print(f"loaded actor.* (strict) from {ckpt_path}")
    print(f"output shape: {tuple(z.shape)} (expect (1, 12))")
    print(f"zero-input  output range: [{z.min():.3f}, {z.max():.3f}]")
    print(f"rand-input  output range: [{r.min():.3f}, {r.max():.3f}] (expect within +/-4)")
    print(f"4*tanh wrapper active: {torch.allclose(z, 4 * torch.tanh(raw), atol=1e-6)}")
    print(f"saved -> {out_path}")


if __name__ == "__main__":
    main()
