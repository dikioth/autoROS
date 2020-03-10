@dataclass
class Transform:
    x: float
    y: float
    z: float


@dataclass
class Rotation:
    yaw: float
    pitch: float
    roll: float


@dataclass
class IMUData:
    rotation: Rotation
    real_acceleration: Transform
    world_acceleration: Transform


@dataclass
class LocationData:
    x: float  # float?
    y: float
    z: float
    quality: float

    def get_as_dict(self):
        return asdict(self)


@dataclass
class Anchor:
    anchor_id: str  # could add = '' but have to rearrange order or do to all
    position: LocationData(-99.0, -99.0, -99.0, -99.0)
    distance: float
    distance_quality: float

    def get_as_dict(self):
        return asdict(self)


@dataclass
class Measurement:
    result_tag: LocationData
    result_imu: IMUData


@dataclass
class Target:
    x: float
    y: float
    yaw: float
    velocity: float = 0


@dataclass
class ControlSignal:
    velocity: float = 0
    steering: float = 0
    target: Target = Target(0, 0, 0, 0)
    error: Target = Target(0, 0, 0, 0)

    def to_numpy(self):
        return np.array([self.velocity, self.steering])


@dataclass
class Context:
    new_measurement_event: Event
    new_estimated_state_event: Event
    new_control_signal_event: Event
    settings: dict
    to_web_queue: Queue
    from_web_queue: Queue
    measurement: (LocationData, IMUData) = None
    control_signal = ControlSignal(0, 0, Target(0, 0, 0, 0))
    estimated_state: EstimatedState = None
    _auto_steering = False

    @property
    def auto_steering(self):
        return self._auto_steering

    @auto_steering.setter
    def auto_steering(self, value):
        if value:
            print("Auto steering enabled.")
        else:
            print("Auto steering disabled.")

    def __init__(self, settings_file=None):

        self.new_control_signal_event = Event()
        self.new_estimated_state_event = Event()
        self.new_measurement_event = Event()
        self.to_web_queue = Queue()
        self.from_web_queue = Queue()

        self.new_control_signal_event.set()

        if settings_file is None:
            self.settings = load_default_settings()
        else:
            self.settings = load_settings(settings_file)

    async def get_estimated_state(self):
        await self.new_estimated_state_event.wait()
        return self.estimated_state


@dataclass
class EstimatedState:
    location_est: LocationData
    x_v_est: float
    y_v_est: float
    log_likelihood: float
    likelihood: float
    x_acc_est: float
    y_acc_est: float
    yaw_est: float
    yaw_acc_est: float
    measurement: Measurement = None  # has result_tag, result_imu
