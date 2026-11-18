"""Local UDP bursts retain real packets without firmware, shared fleet ports or GUI."""

import socket
import time

from integrations.mavlink.impairment import SimulatedLink


def test_api_proxy_absorbs_a_bounded_telemetry_burst() -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as application:
        application.bind(("127.0.0.1", 0))
        application.settimeout(0.1)
        link = SimulatedLink(0, application.getsockname()[1])
        try:
            destination = link.socket.getsockname()
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as publisher:
                # Queue a short real burst before scheduling the proxy. The former
                # default Linux receive buffer dropped part of this burst.
                expected = {index.to_bytes(4, "big") + b"x" * 124 for index in range(300)}
                for packet in sorted(expected):
                    publisher.sendto(packet, destination)
                link.start()
                received = set()
                deadline = time.monotonic() + 3
                while len(received) < len(expected) and time.monotonic() < deadline:
                    try:
                        packet, _ = application.recvfrom(65535)
                        received.add(packet)
                    except socket.timeout:
                        continue
            assert received == expected
            assert link.snapshot()["dropped_packets"] == 0
            assert link.snapshot()["queued_packets"] == 0
        finally:
            link.close()
