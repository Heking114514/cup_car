import unittest

from car_serial_gui.protocol import EncoderFrame, VelocityCommand, parse_encoder_frame


class ProtocolTests(unittest.TestCase):
    def test_velocity_encoding(self) -> None:
        self.assertEqual(VelocityCommand(0.25, -1.5).encode(), "0.250,-1.500")

    def test_encoder_frame(self) -> None:
        self.assertEqual(
            parse_encoder_frame("ENC,1250,125,123,-45\r\n"),
            EncoderFrame(123, -45, mcu_time_ms=1250, sequence=125),
        )

    def test_legacy_encoder_frame(self) -> None:
        self.assertEqual(parse_encoder_frame("ENC,123,-45\r\n"), EncoderFrame(123, -45))

    def test_invalid_frame(self) -> None:
        self.assertIsNone(parse_encoder_frame("LOG,123,-45"))


if __name__ == "__main__":
    unittest.main()
