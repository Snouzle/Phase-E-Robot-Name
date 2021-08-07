#include "RobotState.hpp"

void InitialState::process(MotionPlanRunner &runner) {
    runner.printState();

    std::unique_ptr<RobotState> state {std::make_unique<ReadState>()};
    runner.setState(state);
}

TurningState::TurningState(const double &vd, const double &ad,
                           MotionPlanRunner &runner):
    RobotState(), mPt{runner.getHeadingAngle()}, mVd{vd}, mAd{ad}, mVp{runner.getLeftMotor()} {}

void TurningState::process(MotionPlanRunner &runner) {
    double pc {runner.getIMU()[2]};
    double ts {runner.getTimeStep()};
    double pt {0};

    // If robot target is +ve and current position is -ve but the
    // robot is turning clockwise, we must decrease target position
    // by 2PI, and vice versa
    if (abs(mPt - pc) > PI) pt = mPt + sign(pc - mPt) * 2 * PI;
    else pt = mPt;

    // Implement PID Controller (Yaw input, motor velocity output)
    double error {pt - pc};
    mErrorIntegral += error * ts;
    double errorDerivative = (mPreviousError - error) / ts;

    double vc {KP * error + KD * errorDerivative + KI * mErrorIntegral};

    // Set acceleration and maximum velocity
    if (abs(vc) > mVd)
        vc = (sign(vc)) * mVd;
    if (mAd != -1) {
        double a {(vc - mVp) / ts};
        if (abs(a) > mAd) 
            a = sign(a) * mAd;
        vc = mVp + a * ts;
        mVp = vc;
    }

    // Turning finishing condition
    if (abs(vc) < 0.01) {
        auto sensors = runner.getMotorSensors();
        runner.setTargetPosition(sensors[0], sensors[1]);
        runner.printState();
        std::unique_ptr<RobotState> state {std::make_unique<ReadState>()};
        runner.setState(state);
    } else {
        runner.setLeftMotor(-vc);
        runner.setRightMotor(vc);
    }
}

void ReadState::process(MotionPlanRunner &runner) {
    char motion {runner.getNextMotion()};
    std::unique_ptr<RobotState> state;
    auto motorPositions {runner.getTargetPosition()};

    switch (motion) {
        case (char)Direction::LEFT:
            runner.setTargetPosition(INFINITY, INFINITY);
            runner.moveRobot(0, 0, MotionPlanRunner::ChangeHeading::LEFT);
            state = std::make_unique<TurningState>(0.5*MAX_SPEED, 0.5, runner);
            runner.setState(state);
			break;
		case (char)Direction::FORWARD:
            runner.setTargetPosition(motorPositions[0] + FORWARD_RADIANS, motorPositions[1] + FORWARD_RADIANS);
            runner.moveRobot(MAX_SPEED, MAX_SPEED,
                             MotionPlanRunner::ChangeHeading::FORWARD);
            runner.updatePosition();
            state = std::make_unique<RunningState>();
            runner.setState(state);
			break;
		case (char)Direction::RIGHT:
            runner.setTargetPosition(INFINITY, INFINITY);
            runner.moveRobot(0, 0, MotionPlanRunner::ChangeHeading::RIGHT);
            state = std::make_unique<TurningState>(0.5*MAX_SPEED, 0.5, runner);
            runner.setState(state);
			break;
        case '\0':
            state = std::make_unique<FinishedState>(runner);
            runner.setState(state);
            break;
		default:
            break;
	}
}

void RunningState::process(MotionPlanRunner &runner) {
    auto motorSensors {runner.getMotorSensors()};

    /**
    if (there is a wall) {
        replan()
        return
    }
    **/

    // Moving forward finishing condition
    if (motorSensors[0] == mPrevLeftSensor && 
        motorSensors[1] == mPrevRightSensor) {
        runner.printState();
        std::unique_ptr<RobotState> state {std::make_unique<ReadState>()};
        runner.setState(state);
    } else {
        mPrevLeftSensor = motorSensors[0];
        mPrevRightSensor = motorSensors[1];
    }
}

FinishedState::FinishedState(MotionPlanRunner &runner): RobotState() {
    runner.finishRobot();
}

int sign(const double &val) {
    return (val > 0) - (val < 0);
}