/* ODRIVE */

constexpr float MINIMUM_NON_SHAKE_SPEED = 4.f; // turns/s
constexpr float MAXIMUM_SPEED = 50.f;          // TODO: verify, turns/s

/* DRIVE */

constexpr float GEAR_RATIO = 0.25f;    // reduction ratio of motor speed to wheel speed
constexpr float WHEEL_RADIUS = 0.026f; // m
constexpr float TRACK_WIDTH = 0.153f;  // m

/* POSE */

constexpr float POSE_ALPHA = 0.5f; // how much to trust the algorithm vs the sensors (0.f = all sensors, 1.f = all algorithm)

/* CONTROL */

constexpr float MAX_LINEAR_VEL = 0.5f;  // m/s
constexpr float MAX_ANGULAR_VEL = 0.5f; // rad/s

constexpr float DEFAULT_MC_LINEAR_GAIN = 1.0f;
constexpr float DEFAULT_MC_ANGULAR_GAIN = 2.0f;
constexpr float DEFAULT_MC_THETA_GAIN = -0.5f;