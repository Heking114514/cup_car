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
    mcu_time_ms: int | None = None
    sequence: int | None = None


def parse_encoder_frame(line: str) -> EncoderFrame | None:
    fields = [field.strip() for field in line.strip().split(",")]
    if not fields or fields[0].upper() != "ENC":
        return None

    try:
        if len(fields) == 5:
            return EncoderFrame(
                left=int(fields[3]),
                right=int(fields[4]),
                mcu_time_ms=int(fields[1]),
                sequence=int(fields[2]),
            )
        if len(fields) == 3:
            return EncoderFrame(left=int(fields[1]), right=int(fields[2]))
    except (ValueError, IndexError):
        return None
    return None
