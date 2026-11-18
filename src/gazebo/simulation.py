"""Physics lifecycle adapter with an isolated, persistent transport connection."""

from gazebo.transport import GazeboControlConnection


class GazeboSimulation:
    def __init__(
        self,
        model_name: str = "x500_0",
        spawn_enu_m: tuple = (0, 0, 0.1),
        connection: GazeboControlConnection | None = None,
    ) -> None:
        self.model_name = model_name
        self.spawn_enu_m = spawn_enu_m
        self.owns_connection = connection is None
        self.connection = connection or GazeboControlConnection()

    def start(self) -> None:
        self.connection.request("resume")

    def pause(self) -> None:
        self.connection.request("pause")

    def stop(self) -> None:
        self.pause()

    def reset(self) -> None:
        # Preserve the sensor clock; Harmonic does not implement model_only reset.
        self.start()
        self.connection.request(
            "restore_pose",
            {
                "model_name": self.model_name,
                "position_enu_m": self.spawn_enu_m,
            },
        )

    def close(self) -> None:
        if self.owns_connection:
            self.connection.close()

    def remove(self) -> None:
        self.connection.request("remove_model", {"model_name": self.model_name})
