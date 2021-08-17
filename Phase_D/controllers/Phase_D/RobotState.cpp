#include "RobotState.hpp"

void InitialState::process(MotionStrategy &runner) {
    runner.processState();

    std::unique_ptr<RobotState> state {std::make_unique<ReadState>()};
    runner.setState(state);
}

TurningState::TurningState(const double &vd, const double &ad,
                           MotionStrategy &runner):
    RobotState(), mPt{runner.getHeadingAngle()}, mVd{vd}, mAd{ad}, mVp{runner.getLeftMotor()} {}

void TurningState::process(MotionStrategy &runner) {
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
        std::unique_ptr<RobotState> state {std::make_unique<ReadState>()};
        runner.setState(state);
        runner.setLeftMotor(0);
        runner.setRightMotor(0);
        runner.processState();
    } else {
        runner.setLeftMotor(-vc);
        runner.setRightMotor(vc);
    }
}

void ReadState::process(MotionStrategy &runner) {
    char motion {runner.getNextMotion()};
    std::unique_ptr<RobotState> state;
    auto motorPositions {runner.getTargetPosition()};
    int repeats;

    switch (motion) {
        case (char)Direction::LEFT:
            runner.setTargetPosition(INFINITY, INFINITY);
            runner.moveRobot(0, 0, MotionStrategy::ChangeHeading::LEFT);
            state = std::make_unique<TurningState>(0.5*MAX_SPEED, 0.5, runner);
            runner.setState(state);
			break;
		case (char)Direction::FORWARD:
            repeats = runner.getNumRepeat(motion);
            runner.setTargetPosition(motorPositions[0] + repeats * FORWARD_RADIANS, 
                                     motorPositions[1] + repeats * FORWARD_RADIANS);
            runner.moveRobot(MAX_SPEED, MAX_SPEED,
                             MotionStrategy::ChangeHeading::FORWARD);
            runner.updatePosition(repeats);
            state = std::make_unique<RunningState>(motorPositions[0], motorPositions[1], repeats);
            runner.setState(state);
			break;
		case (char)Direction::RIGHT:
            runner.setTargetPosition(INFINITY, INFINITY);
            runner.moveRobot(0, 0, MotionStrategy::ChangeHeading::RIGHT);
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

RunningState::RunningState(const double &leftSensor, const double &rightSensor,
                           const int &repeats):
    RobotState(), mStartLeftSensor{leftSensor}, mStartRightSensor{rightSensor},
    mLeftTarget{leftSensor + repeats*FORWARD_RADIANS}, 
    mRightTarget{rightSensor + repeats*FORWARD_RADIANS} {
    float d {(float)((float)(repeats) * FORWARD_RADIANS)};
    float t_to_max = (MAX_SPEED) / mAMax;
    float d_to_max = (MAX_SPEED) * t_to_max / 2;
    times[0] = 0;

    if (d_to_max * 2 > d) {
        times[3] = 2 * sqrt(d / mAMax);
        times[1] = (times[2] = (times[3]/2));
    } else {
        times[1] = t_to_max;
        times[2] = times[1] + (d - 2 * d_to_max) / MAX_SPEED;
        times[3] = t_to_max + times[2];
    }
}

void RunningState::bangBang(MotionStrategy &runner) {
    mT += (float)runner.getTimeStep() / 1000.0;
    float vc{0.0};
    if (mT >= times[3]) {
        std::unique_ptr<RobotState> state {std::make_unique<ReadState>()};
        runner.setState(state);
        runner.setLeftMotor(0);
        runner.setRightMotor(0);
        runner.processState();
        return;
    } else if (mT < times[1]) {
        vc = mAMax * mT;
    } else if (mT >= times[2]) {
        vc = -mAMax * (mT - times[3]);
    } else {
        vc = MAX_SPEED-0.001;
    }

    runner.setLeftMotor(vc);
    runner.setRightMotor(vc);
}
    
void RunningState::webotsController(MotionStrategy &runner) {
    auto motorSensors {runner.getMotorSensors()};
    if ((motorSensors[0] - mLeftTarget < 0.1) && 
        (motorSensors[1] - mPrevRightSensor < 0.1) &&
        (motorSensors[0] == mPrevLeftSensor) &&
        (motorSensors[1] == mPrevRightSensor)) {
        runner.processState();
        std::unique_ptr<RobotState> state {std::make_unique<ReadState>()};
        runner.setState(state);
    } else {
        mPrevLeftSensor = motorSensors[0];
        mPrevRightSensor = motorSensors[1];
    }
}

void RunningState::process(MotionStrategy &runner) {
    auto motorSensors {runner.getMotorSensors()};

    double frontSensor {runner.getDistanceSensors()[(int)MotionStrategy::Wall::FRONT]};

    if (frontSensor < 350) {
        if (!replan) {
            int moves {(int)floor((mLeftTarget - motorSensors[0])/FORWARD_RADIANS)};
            mLeftTarget = motorSensors[0] - fmod(motorSensors[0] - mStartLeftSensor, FORWARD_RADIANS);;
            mRightTarget = motorSensors[1] - fmod(motorSensors[1] - mStartRightSensor, FORWARD_RADIANS);
            runner.setTargetPosition(mLeftTarget, mRightTarget);
            runner.updatePosition(-moves);
            runner.moveRobot(MAX_SPEED, MAX_SPEED,
                             MotionStrategy::ChangeHeading::FORWARD);
            runner.replan();
            replan = true;
        }
    } else {
        // replan = false;
        if (runner.getTrajectory() == MotionStrategy::Trajectory::BANG_BANG && !replan)
        // Moving forward finishing condition
            bangBang(runner);
        else 
            webotsController(runner);
    }
}

FinishedState::FinishedState(MotionStrategy &runner): RobotState() {
    runner.finishRobot();
}

int sign(const double &val) {
    return (val > 0) - (val < 0);
}