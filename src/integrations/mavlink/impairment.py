"""Bounded, simulation-only UDP link used by the application's MAVLink adapter."""

import heapq
import random
import socket
import threading
import time


class SimulatedLink:
    """Impair both directions of the API link; QGC has a separate unaffected link."""

    def __init__(self, vehicle_port: int = 14540, application_port: int = 14542) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Full-speed SITL emits short telemetry bursts while Python mapping is
        # scheduled. Bound buffering explicitly instead of losing estimates in
        # the kernel; downstream timestamp/pose-gap checks remain unchanged.
        # Kernels may clamp this request, so expose the effective allocation.
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self.socket.bind(("127.0.0.1", vehicle_port))
        self.socket.settimeout(0.02)
        self.application_address = ("127.0.0.1", application_port)
        self.vehicle_address: tuple[str, int] | None = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.latency_ms = 0
        self.loss_percent = 0
        self.seed = 0
        self.random = random.Random(0)
        self.queue: list[tuple] = []
        self.sequence = 0
        self.received = 0
        self.dropped = 0
        self.sent = 0
        self.thread = threading.Thread(target=self.run, name="simulated-mavlink-link", daemon=True)

    def configure(self, latency_ms: int, loss_percent: int, seed: int) -> None:
        if type(latency_ms) is not int or not 0 <= latency_ms <= 2000:
            raise ValueError("Link latency must be an integer in 0..2000 ms")
        if type(loss_percent) is not int or not 0 <= loss_percent <= 100:
            raise ValueError("Packet loss must be an integer in 0..100 percent")
        if type(seed) is not int or not 0 <= seed <= 2**31 - 1:
            raise ValueError("Link seed must be an integer in 0..2147483647")
        with self.lock:
            self.latency_ms, self.loss_percent, self.seed = latency_ms, loss_percent, seed
            self.random = random.Random(seed)
            self.queue.clear()

    def snapshot(self) -> dict:
        with self.lock:
            return {
                "receive_buffer_bytes": self.socket.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
                "latency_ms": self.latency_ms,
                "loss_percent": self.loss_percent,
                "seed": self.seed,
                "received_packets": self.received,
                "dropped_packets": self.dropped,
                "sent_packets": self.sent,
                "queued_packets": len(self.queue),
                "scope": "Bidirectional application MAVLink link; QGroundControl is separate",
            }

    def start(self) -> None:
        self.thread.start()

    def close(self) -> None:
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=3)
        self.socket.close()

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                packet, sender = self.socket.recvfrom(65535)
            except socket.timeout:
                packet = None
            if packet is not None:
                if sender == self.application_address:
                    destination = self.vehicle_address
                else:
                    self.vehicle_address = sender
                    destination = self.application_address
                with self.lock:
                    self.received += 1
                    if destination is None or len(self.queue) >= 1024:
                        self.dropped += 1
                    elif self.random.random() * 100 < self.loss_percent:
                        self.dropped += 1
                    else:
                        self.sequence += 1
                        heapq.heappush(
                            self.queue,
                            (
                                time.monotonic() + self.latency_ms / 1000,
                                self.sequence,
                                packet,
                                destination,
                            ),
                        )
            with self.lock:
                while self.queue and self.queue[0][0] <= time.monotonic():
                    _, _, packet, destination = heapq.heappop(self.queue)
                    self.socket.sendto(packet, destination)
                    self.sent += 1
