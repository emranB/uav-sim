"""Coordinate panel use cases through transport-independent component owners."""

from pathlib import Path

from common.configuration.environment import EnvironmentControls
from common.configuration.mission_store import MissionStore
from common.configuration.missions import parse_plan
from common.interfaces.vehicles import VehicleAdapter
from common.logging.records import RecordingCatalog


class WorkspaceFeatures:
    def __init__(
        self,
        vehicle: VehicleAdapter,
        directory: Path,
        world: dict,
        configuration: dict,
        runtime_directory: Path = Path("/runtime"),
        shared_directory: Path | None = None,
    ) -> None:
        self.vehicle = vehicle
        self.environment = EnvironmentControls(
            vehicle, directory / "logs", runtime_directory, configuration["vehicle_id"]
        )
        self.missions = MissionStore(directory)
        self.recordings = RecordingCatalog(directory, shared_directory)
        self.world = world
        self.configuration = configuration
        self.mission_name: str | None = None
        self.vehicle_parameters: dict = {}

    def read(self, resource: str, query: dict[str, list[str]]) -> dict:
        if resource == "workspace":
            return {
                "environment": self.environment.snapshot(),
                "world": self.world,
                "vehicle": self.configuration,
                "mission_name": self.mission_name,
                "vehicle_parameters": self.vehicle_parameters,
            }
        if resource == "missions":
            return {"items": self.missions.list()}
        if resource == "experiments":
            return {"items": self.missions.list("experiments")}
        if resource == "recordings":
            return self.recordings.list()
        if resource == "logs":
            return self.recordings.logs(
                query.get("source", ["px4"])[0], query.get("query", [""])[0]
            )
        raise ValueError(f"Unknown workspace resource: {resource}")

    def execute(self, operation: str, body: dict) -> dict:
        if operation.startswith(("sensors/", "interference/")):
            return self.environment.execute(operation, body)
        if operation == "recordings/compare":
            return self.recordings.compare(body.get("left"), body.get("right"))
        if operation == "vehicles/read":
            self.vehicle_parameters = self.vehicle.read_configuration()
            return {"accepted": True, "parameters": self.vehicle_parameters}
        if operation == "vehicles/configure":
            if self.vehicle.snapshot()["armed"]:
                raise RuntimeError("Land before changing vehicle limits")
            self.vehicle_parameters = self.vehicle.configure_limits(
                body.get("cruise_speed_m_s"), body.get("climb_speed_m_s")
            )
            return {"accepted": True, "parameters": self.vehicle_parameters}
        name = body.get("name")
        if operation in {"missions/load", "experiments/load"} and self.vehicle.snapshot()["armed"]:
            raise RuntimeError("Land before loading a mission or experiment")
        if operation == "missions/save":
            self.missions.save(name, self.vehicle.download())
            self.mission_name = name
        elif operation == "missions/load":
            self.vehicle.upload(parse_plan(self.missions.load(name)))
            self.mission_name = name
        elif operation == "missions/export":
            return {"accepted": True, "mission": self.missions.load(name)}
        elif operation == "missions/archive":
            self.missions.archive(name)
        elif operation == "experiments/archive":
            self.missions.archive(name, "experiments")
        elif operation == "experiments/save":
            self.missions.save_experiment(
                name,
                {
                    "mission": self.vehicle.download(),
                    "world": self.world,
                    "vehicle": self.configuration,
                    "environment": self.environment.snapshot(),
                    "versions": self.versions(),
                    "vehicle_parameters": self.vehicle.read_configuration(),
                    "repeatability": "Not measured",
                },
            )
        elif operation == "experiments/load":
            experiment = self.missions.load_experiment(name)
            if experiment["world"] != self.world:
                raise ValueError("Experiment world differs; configure and restart that world first")
            self.environment.execute("interference/clear", {})
            parameters = experiment["vehicle_parameters"]
            self.vehicle_parameters = self.vehicle.configure_limits(
                parameters["cruise_speed_m_s"], parameters["climb_speed_m_s"]
            )
            self.vehicle.upload(parse_plan(experiment["mission"]))
            settings = experiment["environment"]
            self.environment.execute("interference/gnss", {"satellites": settings["satellites"]})
            self.environment.execute("sensors/gnss", {"enabled": settings["gnss_enabled"]})
            self.environment.execute("interference/link", settings["link"])
            self.mission_name = f"Experiment: {name}"
        else:
            raise ValueError(f"Unsupported workspace operation: {operation}")
        return {"accepted": True, "name": name, "operation": operation}

    def reset(self) -> None:
        self.environment.clear()
        self.mission_name = None
        self.vehicle_parameters = {}

    @staticmethod
    def versions() -> dict[str, str]:
        """Record the upstream revisions and any applied native firmware patches."""
        directory = Path("/opt/px4/build/px4_sitl_default")
        return {
            name: (directory / name).read_text().strip()
            for name in ("source-revision", "models-revision", "firmware-patches.sha256")
            if (directory / name).exists()
        }
