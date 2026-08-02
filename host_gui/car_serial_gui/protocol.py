from dataclasses import dataclass


@dataclass(frozen=True)
class VelocityCommand:
    vx: float
    az: float

    def encode(self) -> str:
        return f"{self.vx:.3f},{self.az:.3f}"


@dataclass(frozen=True)
class EncoderFrame:
    left: int
    right: int


def parse_encoder_frame(line: str) -> EncoderFrame | None:
    fields = [field.strip() for field in line.strip().split(",")]
    if len(fields) != 3 or fields[0].upper() != "ENC":
        return None

    try:
        return EncoderFrame(left=int(fields[1]), right=int(fields[2]))
    except ValueError:
        return None
