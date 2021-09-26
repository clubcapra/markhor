#pragma once
#include <stdint.h>
namespace ctre {
	namespace phoenix {
		namespace motion {
			/**
			 * Motion Profile Trajectory Point
			 * This is simply a data transfer object.
			 */
			struct TrajectoryPoint {
				/** The position to servo to (in sensor units). */
				double position = 0;
				/** The velocity to feed-forward (in sensor-units per 100ms). */
				double velocity = 0;
				/** Added to the output of PID[0], should be within [-1,+1] where 0.01 = 1%. */
				double arbFeedFwd = 0;
				/** Not used.  Use auxiliaryPos instead.  @see auxiliaryPos */
				double headingDeg = 0;
				/** The position for auxiliary PID[1] to target (in sensor units). */
				double auxiliaryPos = 0;
				/** The velocity for auxiliary PID[1] to target. (in sensor-units per 100ms). */
				double auxiliaryVel = 0;
				/** Added to the output of PID[1], should be within [-1,+1] where 0.01 = 1%. */
				double auxiliaryArbFeedFwd = 0;

				/**
				 * Which slot to get PIDF gains.
				 * PID is used for position servo.
				 * F is used as the Kv constant for velocity feed-forward.
				 * Typically this is hard-coded
				 * to a particular slot, but you are free to gain schedule if need be.
				 * gain schedule if need be.
				 * Choose from [0,3].
				 */
				uint32_t profileSlotSelect0 = 0;

				/**
				 * Which slot to get PIDF gains for auxiliary PID.
				 * This only has impact during MotionProfileArc Control mode.
				 * Choose from [0,3].
				 */
				uint32_t profileSlotSelect1 = 0;
				/**
				 * Set to true to signal Talon that this is the final point, so do not
				 * attempt to pop another trajectory point from out of the Talon buffer.
				 * Instead continue processing this way point.  Typically the velocity
				 * member variable should be zero so that the motor doesn't spin indefinitely.
				 */
				bool isLastPoint = false;
				/**
				 * Set to true to signal Talon to zero the selected sensor.
				 * When generating MPs, one simple method is to make the first target position zero,
				 * and the final target position the target distance from the current position.
				 * Then when you fire the MP, the current position gets set to zero.
				 * If this is the intent, you can set zeroPos on the first trajectory point.
				 *
				 * Otherwise you can leave this false for all points, and offset the positions
				 * of all trajectory points so they are correct.
				 *
				 * If using multiple sensor sources (Arc modes) we recommend you manually set sensor positions
				 * before arming MP.
				 */
				bool zeroPos = false;

				/**
				 * Duration (ms) to apply this trajectory pt.
				 * This time unit is ADDED to the existing base time set by
				 * ConfigMotionProfileTrajectoryPeriod().
				 */
				int timeDur = 0;

				/**
				 * If using MotionProfileArc, this flag must be true on all points.
				 * If using MotionProfile, this flag must be false on all points.
				 */
				bool useAuxPID = false;

				TrajectoryPoint() {
					/* initializers above */
				}

				/**
				 * Create a trajectory point with specified values
				 * 
				 * @param position The position to servo to (in sensor units).
				 * @param velocity The velocity to feed-forward (in sensor-units per 100ms). 
				 * @param arbFeedFwd Added to the output of PID[0], should be within [-1,+1] where 0.01 = 1%.
				 * @param auxiliaryPos The position for auxiliary PID[1] to target (in sensor units).
				 * @param auxiliaryVel The velocity for auxiliary PID[1] to target. (in sensor-units per 100ms).
				 * @param auxiliaryArbFeedFwd Added to the output of PID[1], should be within [-1,+1] where 0.01 = 1%.
				 * @param profileSlotSelect0 The slot to select for base PIDF gains
				 * @param profileSlotSelect1 The slot to select for auxiliary PIDF gains
				 * @param isLastPoint True if this is the last piont
				 * @param zeroPos True if the motor controller should zero the sensor position
				 * @param timeDur The time duration of this point
				 * @param useAuxPID Enables the auxiliary PID
				 */

				TrajectoryPoint(double position, 
								double velocity, 
								double arbFeedFwd,
								double auxiliaryPos, 
								double auxiliaryVel, 
								double auxiliaryArbFeedFwd, 
								uint32_t profileSlotSelect0, 
								uint32_t profileSlotSelect1,
								bool isLastPoint,
								bool zeroPos, 
								uint32_t timeDur,
								bool useAuxPID) {

					this->position = position;
					this->velocity = velocity;
					this->arbFeedFwd = arbFeedFwd;
					this->auxiliaryPos = auxiliaryPos;
					this->auxiliaryVel = auxiliaryVel;
					this->auxiliaryArbFeedFwd = auxiliaryArbFeedFwd;
					this->profileSlotSelect0 = profileSlotSelect0;
					this->profileSlotSelect1 = profileSlotSelect1;
					this->isLastPoint = isLastPoint;
					this->zeroPos = zeroPos;
					this->timeDur = timeDur;
					this->useAuxPID = useAuxPID;
				}
			};
		} // namespace motion
	} // namespace phoenix
} // namespace ctre
