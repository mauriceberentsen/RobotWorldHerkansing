#include "Robot.hpp"
#include <sstream>
#include <ctime>
#include <chrono>
#include "Thread.hpp"
#include "MathUtils.hpp"
#include "Shape2DUtils.hpp"
#include "Logger.hpp"
#include "Goal.hpp"
#include "WayPoint.hpp"
#include "Wall.hpp"
#include "RobotWorld.hpp"
#include "CommunicationService.hpp"
#include "Client.hpp"
#include "Message.hpp"
#include "MainApplication.hpp"
#include "LaserDistanceSensor.hpp"
#include <stdlib.h>

namespace Model
{

	/**
	 *
	 */
	Robot::Robot() :
								name( ""),
								size( DefaultSize),
								position( DefaultPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false)
	{
		std::shared_ptr< AbstractSensor > proximitySensor( new ProximitySensor( this));
		attachSensor( proximitySensor);
	}
	/**
	 *
	 */
	Robot::Robot( const std::string& aName) :
								name( aName),
								size( DefaultSize),
								position( DefaultPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false)
	{
		std::shared_ptr< AbstractSensor > proximitySensor( new ProximitySensor( this));
		attachSensor( proximitySensor);
	}
	/**
	 *
	 */
	Robot::Robot(	const std::string& aName,
					const Point& aPosition) :
								name( aName),
								size( DefaultSize),
								position( aPosition),
								front( 0, 0),
								speed( 0.0),
								acting(false),
								driving(false),
								communicating(false)
	{
		std::shared_ptr< AbstractSensor > proximitySensor( new ProximitySensor( this));
		attachSensor( proximitySensor);
	}
	/**
	 *
	 */
	Robot::~Robot()
	{
		Application::Logger::log(std::to_string(driving) + "  " + std::to_string(acting) + "  " +std::to_string(communicating));
		if(driving)
		{
			stopDriving();
		}
		if(acting)
		{
			stopActing();
		}
		if(communicating)
		{
			stopCommunicating();
		}
	}
	int Robot::randomNumberBetweenUpToN(int N /*=100 */)
	{
		std::srand(position.x + position.y);
		return std::rand() % N + 1;
	}
	/**
	 *
	 */
	void Robot::setName( const std::string& aName,
						 bool aNotifyObservers /*= true*/)
	{
		name = aName;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}

	}
	/**
	 *
	 */
	Size Robot::getSize() const
	{
		return size;
	}
	/**
	 *
	 */
	void Robot::setSize(	const Size& aSize,
							bool aNotifyObservers /*= true*/)
	{
		size = aSize;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::setPosition(	const Point& aPosition,
								bool aNotifyObservers /*= true*/)
	{
		position = aPosition;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	BoundedVector Robot::getFront() const
	{
		return front;
	}
	/**
	 *
	 */
	void Robot::setFront(	const BoundedVector& aVector,
							bool aNotifyObservers /*= true*/)
	{
		front = aVector;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	float Robot::getSpeed() const
	{
		return speed;
	}
	/**
	 *
	 */
	void Robot::setSpeed( float aNewSpeed,
						  bool aNotifyObservers /*= true*/)
	{
		speed = aNewSpeed;
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::startActing()
	{
		acting = true;
		std::thread newRobotThread( [this]{	startDriving();});
		robotThread.swap( newRobotThread);
	}
	/**
	 *
	 */
	void Robot::stopActing()
	{
		acting = false;
		driving = false;
		robotThread.join();
	}
	/**
	 *
	 */
	void Robot::startDriving()
	{
		driving = true;

		goal = RobotWorld::getRobotWorld().getGoal( "Goal");
		calculateRoute(goal);

		drive();
	}
	/**
	 *
	 */
	void Robot::stopDriving()
	{
		setSpeed(0.0);
		//driving = false;
	}

	void Robot::restartDriving()
	{
		driving = false;
		robotThread.join();
		driving = true;
		std::thread newRobotThread( [this]{	startDriving();});
		robotThread.swap( newRobotThread);
	}
	/**
	 **/
	Region Robot::getRegion() const
	{
		Point translatedPoints[] = { getFrontRight(), getFrontLeft(), getBackLeft(), getBackRight() };
		return Region( 4, translatedPoints);
	}
	/**
	 *
	 */
	#pragma region movement
	bool Robot::intersects( const Region& aRegion) const
	{
		Region region = getRegion();
		region.Intersect( aRegion);
		return !region.IsEmpty();
	}
	/**
	 *
	 */
	Point Robot::getFrontLeft() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalFrontLeft( x, y);
		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point frontLeft( (originalFrontLeft.x - position.x) * std::cos( angle) - (originalFrontLeft.y - position.y) * std::sin( angle) + position.x, (originalFrontLeft.y - position.y) * std::cos( angle)
		+ (originalFrontLeft.x - position.x) * std::sin( angle) + position.y);

		return frontLeft;
	}
	/**
	 *
	 */
	Point Robot::getFrontRight() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalFrontRight( x + size.x, y);
		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point frontRight( (originalFrontRight.x - position.x) * std::cos( angle) - (originalFrontRight.y - position.y) * std::sin( angle) + position.x, (originalFrontRight.y - position.y)
						  * std::cos( angle) + (originalFrontRight.x - position.x) * std::sin( angle) + position.y);

		return frontRight;
	}
	/**
	 *
	 */
	Point Robot::getBackLeft() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalBackLeft( x, y + size.y);

		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point backLeft( (originalBackLeft.x - position.x) * std::cos( angle) - (originalBackLeft.y - position.y) * std::sin( angle) + position.x, (originalBackLeft.y - position.y) * std::cos( angle)
		+ (originalBackLeft.x - position.x) * std::sin( angle) + position.y);

		return backLeft;

	}
	/**
	 *
	 */
	Point Robot::getBackRight() const
	{
		// x and y are pointing to top left now
		int x = position.x - (size.x / 2);
		int y = position.y - (size.y / 2);

		Point originalBackRight( x + size.x, y + size.y);

		double angle = Utils::Shape2DUtils::getAngle( front) + 0.5 * Utils::PI;

		Point backRight( (originalBackRight.x - position.x) * std::cos( angle) - (originalBackRight.y - position.y) * std::sin( angle) + position.x, (originalBackRight.y - position.y) * std::cos( angle)
		+ (originalBackRight.x - position.x) * std::sin( angle) + position.y);

		return backRight;
	}
	/**
	 *
	 */
		/**
	 *
	 */
	#pragma endregion
	#pragma region communication

	void Robot::startCommunicating()
	{
		if(!communicating)
		{
			communicating = true;


			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven( "-local_port"))
			{
				localPort = Application::MainApplication::getArg( "-local_port").value;
			}

			Messaging::CommunicationService::getCommunicationService().runRequestHandler( toPtr<Robot>(),
																						  std::stoi(localPort));
		}
	}
	/**
	 *
	 */
	void Robot::stopCommunicating()
	{
		if(communicating)
		{
			communicating = false;

			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven( "-local_port"))
			{
				localPort = Application::MainApplication::getArg( "-local_port").value;
			}

			Messaging::Client c1ient( 	"localhost",
										localPort,
										toPtr<Robot>());
			Messaging::Message message( 1, "stop");
			c1ient.dispatchMessage( message);
		}
	}
	/**
	 *
	 */
	void Robot::BroadcastPostion()
	{

			std::string remoteIpAdres = "localhost";
			std::string remotePort = "12345";
			Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot( "Robot");
			if(robot)
				{
				if (Application::MainApplication::isArgGiven( "-remote_ip"))
				{
					remoteIpAdres = Application::MainApplication::getArg( "-remote_ip").value;
				}
				if (Application::MainApplication::isArgGiven( "-remote_port"))
				{
					remotePort = Application::MainApplication::getArg( "-remote_port").value;
				}

				// We will request an echo message. The response will be "Hello World", if all goes OK,
				// "Goodbye cruel world!" if something went wrong.
				Messaging::Client c1ient( remoteIpAdres,
										remotePort,
										robot);
				Messaging::Message message( Model::Robot::MessageType::EchoLocation,serializeRobotInfo());
				c1ient.dispatchMessage( message);
			}
			else{
				std::cout<<"Something went wrong"<<std::endl;
			}
		
	}
		/**
	 *
	 */
	void Robot::negotiate()
	{

			std::string remoteIpAdres = "localhost";
			std::string remotePort = "12345";
			Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot( "Robot");
			if(robot)
				{
				if (Application::MainApplication::isArgGiven( "-remote_ip"))
				{
					remoteIpAdres = Application::MainApplication::getArg( "-remote_ip").value;
				}
				if (Application::MainApplication::isArgGiven( "-remote_port"))
				{
					remotePort = Application::MainApplication::getArg( "-remote_port").value;
				}

				// We will request an echo message. The response will be "Hello World", if all goes OK,
				// "Goodbye cruel world!" if something went wrong.
				Messaging::Client c1ient( remoteIpAdres,
										remotePort,
										robot);
				Messaging::Message message( Model::Robot::MessageType::NegotiateRequest,std::to_string(randomNumberBetweenUpToN()));
				c1ient.dispatchMessage( message);
			}
			else{
				std::cout<<"Something went wrong"<<std::endl;
			}
		
	}

	void Robot::drivingAllowed()
	{
			std::string remoteIpAdres = "localhost";
			std::string remotePort = "12345";
			Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot( "Robot");
			if(robot)
				{
				if (Application::MainApplication::isArgGiven( "-remote_ip"))
				{
					remoteIpAdres = Application::MainApplication::getArg( "-remote_ip").value;
				}
				if (Application::MainApplication::isArgGiven( "-remote_port"))
				{
					remotePort = Application::MainApplication::getArg( "-remote_port").value;
				}

				// We will request an echo message. The response will be "Hello World", if all goes OK,
				// "Goodbye cruel world!" if something went wrong.
				Messaging::Client c1ient( remoteIpAdres,
										remotePort,
										robot);
				Messaging::Message message( Model::Robot::MessageType::DriveRequest,"");
				c1ient.dispatchMessage( message);
			}
			else{
				std::cout<<"Something went wrong"<<std::endl;
			}

	}
	void Robot::handleNotification()
	{
		//	std::unique_lock<std::recursive_mutex> lock(robotMutex);

		static int update = 0;
		if ((++update % 200) == 0)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void Robot::handleRequest( Messaging::Message& aMessage)
	{
		switch(aMessage.getMessageType())
		{
			case SyncRequest:
			{
				std::string responseMsgBody = Model::RobotWorld::RobotWorld::getRobotWorld().asSerializedString();
				Application::Logger::log(std::string("Request to sync the world + \n" + aMessage.asString()));
				fillWorld(aMessage.getBody());
				aMessage.setMessageType(SyncResponse);
				aMessage.setBody(responseMsgBody );
				break;
			}
			case EchoRequest:
			{
				Application::Logger::log( __PRETTY_FUNCTION__ + std::string(": EchoRequest"));

				aMessage.setMessageType(EchoResponse);
				aMessage.setBody( ": case 1 " + aMessage.asString());
				break;
			}

			case EchoLocation:
			{
			std::stringstream ss;
			ss << aMessage.getBody();
			unsigned long type;
			std::string aName;
			unsigned long x;
			unsigned long y;
			unsigned long lx;
			unsigned long ly;

			ss >> type >> aName >> x >> y >> lx >> ly ;

			Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot(("_"+aName));

			if(robot){
			robot->setPosition(Point(x,y), true );
			robot->setFront(BoundedVector(lx,ly) , true);
			}
			else Application::Logger::log("_"+aName+"     not found" );

				break;
			}
			case NegotiateRequest:
			{
				stopDriving();
				Application::Logger::log(" someone is near lets negotiate");
				aMessage.setMessageType(NegotiateResponse);
				if(driving)
				{
					std::stringstream ss;
					ss << aMessage.getBody();
					unsigned long OtherRoll;
					ss >> OtherRoll;
					unsigned long OurRoll = randomNumberBetweenUpToN();
					win = (OtherRoll <= OurRoll);
					aMessage.setBody( std::to_string(win));
					Application::Logger::log(" Do we come here?");

				
					if(win) restartDriving();

				}
				else
				{
					aMessage.setBody( std::to_string(false));	
				}
				masterDeterminated = true;
				break;
			}
			case DriveRequest:
			{
				Application::Logger::log("Master arrived I may drive");
				restartDriving();
				aMessage.setMessageType(DriveResponse);
				break;
			}

			case StartRequest:
			{
				if (!acting)
				{
					startActing();
				}
				aMessage.setMessageType(StartResponse);
				break;
			}

			
			default:
			{
				Application::Logger::log(aMessage.asString());

				aMessage.setBody( " default  Goodbye cruel world!");
				break;
			}
		}
	}
	/**
	 *
	 */
	void Robot::handleResponse( const Messaging::Message& aMessage)
	{
		switch(aMessage.getMessageType())
		{
			case SyncResponse:
			{
				Application::Logger::log(std::string("Response to sync" + aMessage.asString()));
				fillWorld(aMessage.getBody());	
				break;
			}
			case EchoResponse:
			{
				Application::Logger::log( __PRETTY_FUNCTION__ + std::string( ": case EchoResponse: not implemented, ") + aMessage.asString());

				break;
			}
			case EchoLocation:
			{
				break;
			}
			case NegotiateResponse:
			{
				Application::Logger::log(" Did we win?  " +  aMessage.getBody());
				std::stringstream ss;
				ss << aMessage.getBody();
				ss >> win;
				/*flip result since it tells the result of the other*/ win = !win;
				masterDeterminated = true;
				if(win) restartDriving();
				break;
			}
			case DriveResponse:
			{
				Application::Logger::log("Other is resuming ");
				break;
			}
			case StartResponse:
			{
				Application::Logger::log("Other is starting ");
				break;
			}

			
			default:
			{
				Application::Logger::log( __PRETTY_FUNCTION__ + std::string( ": default not implemented, ") + aMessage.asString());
				break;
			}
		}
	}
	#pragma endregion

	#pragma region stringfuntions
	std::string Robot::serializeRobotInfo() const
	{
		std::ostringstream os;
	//TODO use enum
		os << "0 " << name << " " << position.x << " " << position.y<<" "  << front.x << " " << front.y;

		return os.str();
	}

	/**
	 *
	 */
	std::string Robot::asString() const
	{
		std::ostringstream os;

		os << "Robot " << name << " at (" << position.x << "," << position.y << ")";

		return os.str();
	}
	/**
	 *
	 */
	std::string Robot::asDebugString() const
	{
		std::ostringstream os;

		os << "Robot:\n";
		os << AbstractAgent::asDebugString();
		os << "Robot " << name << " at (" << position.x << "," << position.y << ")\n";

		return os.str();
	}
	/**
	 *
	 */
	#pragma endregion
	
	void Robot::drive()
	{
		try
		{
			for (std::shared_ptr< AbstractSensor > sensor : sensors)
			{
				sensor->setOn(50);
			}

			if (speed == 0.0)
			{
				speed = 2.0;
			}

			unsigned pathPoint = 0;
			while (position.x > 0 && position.x < 500 && position.y > 0 && position.y < 500 && pathPoint < path.size())
			{
				const PathAlgorithm::Vertex& vertex = path[pathPoint+=speed];
				front = BoundedVector( vertex.asPoint(), position);
				position.x = vertex.x;
				position.y = vertex.y;
				BroadcastPostion();

				if (arrived(goal) && win){
					drivingAllowed();
					notifyObservers();
					win = false;
				} 

				if (arrived(goal) || collision())
				{
					Application::Logger::log(__PRETTY_FUNCTION__ + std::string(": arrived or collision"));
					masterDeterminated = false;
					notifyObservers();
					if(arrived(goal))
					{
						masterDeterminated = false;
					}
					break;
				}

				if(perceptQueue.size() > 0)
				{	
					std::shared_ptr<CollisionPercept> CP = std::dynamic_pointer_cast<CollisionPercept>(perceptQueue.dequeue());
					if(CP->collision && !masterDeterminated)
					{
						stopDriving();
						negotiate();
					}

				}


				notifyObservers();

				std::this_thread::sleep_for( std::chrono::milliseconds( 50));
				// this should be the last thing in the loop
				if(driving == false)
				{
					return;
				}
			} // while

			for (std::shared_ptr< AbstractSensor > sensor : sensors)
			{
				sensor->setOff();
			}
		}
		catch (std::exception& e)
		{
			std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
		}
		catch (...)
		{
			std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
		}
	}

	void Robot::fillWorld(std::string messageBody)
	{
	std::vector<std::string> lines;
	boost::split(lines, messageBody, boost::is_any_of("\n"));



	for(std::string line : lines)
	{
		
		if(!line.empty())
		{
		 Application::Logger::log(&line.at(0));
     	 std::stringstream ss;
     	 std::string Name;
     	 unsigned long X;
     	 unsigned long Y;
     	 unsigned long X2;
     	 unsigned long Y2;



			switch (std::stoi(&line.at(0))) 
			{
				case 0:
					line.erase(line.begin());
					ss << line;

					ss >> Name >> X >> Y;

					Model::RobotWorld::getRobotWorld().newRobot("_"+Name, Point(X, Y),false);


					break;

				case 1:
					line.erase(line.begin());
					ss << line;

					ss >> X >> Y >> X2 >> Y2;
					Model::RobotWorld::getRobotWorld().newWall(Point(X, Y), Point(X2, Y2),false);

					break;

				default:
					Application::Logger::log("Unknown object");
					Application::Logger::log(line);
					break;
				}
			}
		}
	notifyObservers();
	}
	/**
	 *
	 */
	void Robot::calculateRoute(GoalPtr aGoal)
	{
		path.clear();
		if (aGoal)
		{
			// Turn off logging if not debugging AStar
			Application::Logger::setDisable();

			front = BoundedVector( aGoal->getPosition(), position);
			handleNotificationsFor( astar);
			path = astar.search( position, aGoal->getPosition(), size);
			stopHandlingNotificationsFor( astar);

			Application::Logger::setDisable( false);
		}
	}
	/**
	 *
	 */
	bool Robot::arrived(GoalPtr aGoal)
	{
		if (aGoal && intersects( aGoal->getRegion()))
		{
			return true;
		}
		return false;
	}
	/**
	 *
	 */
	bool Robot::collision()
	{
		Point frontLeft = getFrontLeft();
		Point frontRight = getFrontRight();
		Point backLeft = getBackLeft();
		Point backRight = getBackRight();

		const std::vector< WallPtr >& walls = RobotWorld::getRobotWorld().getWalls();
		for (WallPtr wall : walls)
		{
			if (Utils::Shape2DUtils::intersect( frontLeft, frontRight, wall->getPoint1(), wall->getPoint2()) ||
							Utils::Shape2DUtils::intersect( frontLeft, backLeft, wall->getPoint1(), wall->getPoint2())	||
							Utils::Shape2DUtils::intersect( frontRight, backRight, wall->getPoint1(), wall->getPoint2()))
			{
				return true;
			}
		}
		return false;
	}

} // namespace Model
