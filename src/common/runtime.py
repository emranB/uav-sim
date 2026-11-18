"""Minimal mock service: readiness and capabilities, without simulator claims."""

import json
import logging
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from common.configuration.runtime import RuntimeSettings


class BackendServer(ThreadingHTTPServer):
    """Own the requested deployment settings for this backend instance."""

    def __init__(self, settings: RuntimeSettings) -> None:
        self.settings = settings
        super().__init__(("0.0.0.0", 8080), Handler)


class Handler(BaseHTTPRequestHandler):
    server: BackendServer

    def do_GET(self) -> None:
        if self.path not in ("/health", "/capabilities"):
            self.send_error(404)
            return
        response = {
            "status": "ready",
            "mode": "mock",
            "real_simulator": False,
            "service": "control-panel",
            "schema_version": 1,
        }
        if self.path == "/capabilities":
            response["requested_runtime"] = self.server.settings.requested_capabilities()
            response["rendering_validated"] = False
            response["ros_connected"] = False
        body = json.dumps(response).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt: str, *args: object) -> None:
        logging.info(json.dumps({"source": "control-panel", "message": fmt % args}))


def main() -> None:
    settings = RuntimeSettings.from_environment(os.environ)
    destination = settings.output_directory / "logs"
    destination.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(message)s",
        handlers=[logging.StreamHandler(), logging.FileHandler(destination / "backend.log")],
    )
    server = BackendServer(settings)
    logging.info(
        json.dumps(
            {
                "source": "control-panel",
                "event": "startup",
                "mode": "mock",
                "requested_runtime": settings.requested_capabilities(),
            }
        )
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
