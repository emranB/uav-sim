"""Failure evidence retains exact inputs and rejects unbounded data."""

import gzip
import tempfile
import unittest
from pathlib import Path

from common.logging.stereo_diagnostics import write_tracking_failure


class StereoDiagnosticTests(unittest.TestCase):
    def test_loss_inputs_round_trip_and_oversized_pairs_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            images = [b"P6\n1 1\n255\n\x01\x02\x03", b"P6\n1 1\n255\n\x04\x05\x06"]
            result = write_tracking_failure(
                Path(directory), (100, images), (200, images[::-1]), {"tracking_state": 3}
            )
            self.assertEqual(
                gzip.decompress((result / "previous-left.ppm.gz").read_bytes()), images[0]
            )
            self.assertEqual(
                gzip.decompress((result / "current-left.ppm.gz").read_bytes()), images[1]
            )
            with self.assertRaises(ValueError):
                write_tracking_failure(
                    Path(directory),
                    (100, images),
                    (300, [b"x" * 1048577, images[1]]),
                    {"tracking_state": 3},
                )
            self.assertFalse((Path(directory) / "tracking-loss-300").exists())
