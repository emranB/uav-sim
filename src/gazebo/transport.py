"""Bounded requests to an independently owned persistent Gazebo transport worker."""

import multiprocessing
import threading


class GazeboControlConnection:
    def __init__(self) -> None:
        from gazebo.control_worker import run_worker

        context = multiprocessing.get_context("spawn")
        self.connection, child = context.Pipe()
        self.worker = context.Process(target=run_worker, args=(child,), daemon=True)
        self.worker.start()
        child.close()
        self.lock = threading.Lock()

    def request(
        self, operation: str, values: dict | None = None, timeout_seconds: float = 8
    ) -> dict:
        with self.lock:
            if not self.worker.is_alive():
                raise RuntimeError(
                    "Gazebo control worker is not running; restart the control service"
                )
            try:
                self.connection.send({"operation": operation, "values": values or {}})
                if not self.connection.poll(timeout_seconds):
                    # Never let a late response be mistaken for the next operation's ACK.
                    self.worker.terminate()
                    self.worker.join(timeout=1)
                    self.connection.close()
                    raise TimeoutError("Gazebo control worker exceeded its response deadline")
                response = self.connection.recv()
            except TimeoutError:
                raise
            except (OSError, EOFError) as error:
                raise RuntimeError("Gazebo control worker disconnected") from error
            if not response["accepted"]:
                raise RuntimeError(response["error"])
            return response

    def close(self) -> None:
        try:
            self.connection.send(None)
        except (OSError, EOFError):
            pass
        self.worker.join(timeout=1)
        if self.worker.is_alive():
            self.worker.terminate()
            self.worker.join(timeout=2)
        self.connection.close()
