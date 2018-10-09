/*
 * LaserDistanceSensor.cpp
 *
 *  Created on: 15 Oct 2012
 *      Author: jkr
 */

#include "LaserDistanceSensor.hpp"
#include "Robot.hpp"
#include "Logger.hpp"

namespace Model
{
	#pragma region LaserDistanceSensor
	/**
	 *
	 */
	LaserDistanceSensor::LaserDistanceSensor()
	{
	}
	/**
	 *
	 */
	LaserDistanceSensor::LaserDistanceSensor( Robot* aRobot) :
								AbstractSensor( aRobot)
	{
	}
	/**
	 *
	 */
	LaserDistanceSensor::~LaserDistanceSensor()
	{
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractStimulus > LaserDistanceSensor::getStimulus() const
	{
		std::shared_ptr< AbstractStimulus > distanceStimulus( new DistanceStimulus( 666,666));
		return distanceStimulus;
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractPercept > LaserDistanceSensor::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const
	{
		DistanceStimulus* distanceStimulus = dynamic_cast< DistanceStimulus* >( anAbstractStimulus.get());
		return std::shared_ptr< AbstractPercept >( new DistancePercept( distanceStimulus->angle,distanceStimulus->distance));
	}
	/**
	 *
	 */
	std::string LaserDistanceSensor::asString() const
	{
		return "LaserDistanceSensor";
	}
	/**
	 *
	 */
	std::string LaserDistanceSensor::asDebugString() const
	{
		return asString();
	}
	#pragma endregion
	#pragma region ProximitySensor
		/**
	 *
	 */
	ProximitySensor::ProximitySensor()
	{
	}
	/**
	 *
	 */
	ProximitySensor::ProximitySensor( Robot* aRobot) :
								AbstractSensor( aRobot),_Robot(aRobot)
	{
	}
	/**
	 *
	 */
	ProximitySensor::~ProximitySensor()
	{
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractStimulus > ProximitySensor::getStimulus() const
	{
		std::shared_ptr< AbstractStimulus > collisionStimulus( new CollisionStimulus(collision()));
		return collisionStimulus;
	}
	/**
	 *
	 */
	std::shared_ptr< AbstractPercept > ProximitySensor::getPerceptFor( std::shared_ptr< AbstractStimulus > anAbstractStimulus) const
	{
		CollisionStimulus* collisionStimulus = dynamic_cast< CollisionStimulus* >( anAbstractStimulus.get());
		return std::shared_ptr< AbstractPercept >( new CollisionPercept( collisionStimulus->collision));
	}
	/**
	 *
	 */
	std::string ProximitySensor::asString() const
	{
		return "ProximitySensor";
	}
	/**
	 *
	 */
	std::string ProximitySensor::asDebugString() const
	{
		return asString();
	}

	Region ProximitySensor::getRegion() const
	{
		Point translatedPoints[] = { getFrontRight(), getFrontLeft(), getBackLeft(), getBackRight() };
		return Region( 4, translatedPoints);
	}
	/**
	 *
	 */

	bool ProximitySensor::intersects( const Region& aRegion) const
	{
		Region region = getRegion();
		region.Intersect( aRegion);
		return !region.IsEmpty();
	}
	#pragma region getPoint
	/**
	 *
	 */
	Point ProximitySensor::getFrontLeft() const
	{
		// x and y are pointing to top left now
		int x = _Robot->position.x - (_Robot->size.x / 2);
		int y = _Robot->position.y - (_Robot->size.y /2 );

		Point originalFrontLeft( x - 0.1* _Robot->size.x, y - 1.5 * _Robot->size.y);
		double angle = Utils::Shape2DUtils::getAngle( _Robot->front) + 0.5 * Utils::PI;

		Point frontLeft( (originalFrontLeft.x - _Robot->position.x) * std::cos( angle) - (originalFrontLeft.y - _Robot->position.y) * std::sin( angle) + _Robot->position.x, (originalFrontLeft.y - _Robot->position.y) * std::cos( angle)
		+ (originalFrontLeft.x - _Robot->position.x) * std::sin( angle) + _Robot->position.y);

		return frontLeft;
	}
	/**
	 *
	 */
	Point ProximitySensor::getFrontRight() const
	{
		// x and y are pointing to top left now
		int x = _Robot->position.x - (_Robot->size.x / 2);
		int y = _Robot->position.y - (_Robot->size.y / 2);
		

		Point originalFrontRight( x + 1.1 * _Robot->size.x, y - 1.5 * _Robot->size.y);
		double angle = Utils::Shape2DUtils::getAngle( _Robot->front) + 0.5 * Utils::PI;

		Point frontRight( (originalFrontRight.x - _Robot->position.x) * std::cos( angle) - (originalFrontRight.y - _Robot->position.y) * std::sin( angle) + _Robot->position.x, (originalFrontRight.y - _Robot->position.y)
						  * std::cos( angle) + (originalFrontRight.x - _Robot->position.x) * std::sin( angle) + _Robot->position.y);

		return frontRight;
	}
	/**
	 *
	 */
	Point ProximitySensor::getBackLeft() const
	{
		// x and y are pointing to top left now
		int x = _Robot->position.x - (_Robot->size.x / 2);
		int y = _Robot->position.y - (_Robot->size.y / 2);

		Point originalBackLeft( x - 0.1* _Robot->size.x, y);

		double angle = Utils::Shape2DUtils::getAngle( _Robot->front) + 0.5 * Utils::PI;

		Point backLeft( (originalBackLeft.x - _Robot->position.x) * std::cos( angle) - (originalBackLeft.y - _Robot->position.y) * std::sin( angle) + _Robot->position.x, (originalBackLeft.y - _Robot->position.y) * std::cos( angle)
		+ (originalBackLeft.x - _Robot->position.x) * std::sin( angle) + _Robot->position.y);

		return backLeft;

	}
	/**
	 *
	 */
	Point ProximitySensor::getBackRight() const
	{
		// x and y are pointing to top left now
		int x = _Robot->position.x - (_Robot->size.x / 2);
		int y = _Robot->position.y - (_Robot->size.y / 2);

		Point originalBackRight( x + 1.1 * _Robot->size.x + _Robot->size.x, y );

		double angle = Utils::Shape2DUtils::getAngle( _Robot->front) + 0.5 * Utils::PI;

		Point backRight( (originalBackRight.x - _Robot->position.x) * std::cos( angle) - (originalBackRight.y - _Robot->position.y) * std::sin( angle) + _Robot->position.x, (originalBackRight.y - _Robot->position.y) * std::cos( angle)
		+ (originalBackRight.x - _Robot->position.x) * std::sin( angle) + _Robot->position.y);

		return backRight;
	}
	#pragma endregion

	bool ProximitySensor::collision() const
	{
		Point frontLeft = getFrontLeft();
		Point frontRight = getFrontRight();
		Point backLeft = getBackLeft();
		Point backRight = getBackRight();


		const std::vector<RobotPtr>& robots = RobotWorld::getRobotWorld().getRobots();
		for(RobotPtr otherRobot :  robots)
		{
			if(otherRobot != _Robot)			{
			if (Utils::Shape2DUtils::intersect( frontLeft, frontRight, otherRobot->getFrontLeft(), otherRobot->getFrontRight()) ||
				Utils::Shape2DUtils::intersect( backLeft, backRight, otherRobot->getBackLeft(), otherRobot->getBackRight()) ||
				Utils::Shape2DUtils::intersect( frontLeft, backLeft, otherRobot->getFrontLeft(), otherRobot->getBackRight()) ||
				Utils::Shape2DUtils::intersect( frontRight, backRight, otherRobot->getFrontRight(), otherRobot->getBackRight()))
			{
				return true;
			}

			}
		}

		return false;
	}


	#pragma endregion
} // namespace Model
