#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "Config.hpp"

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "AbstractAgent.hpp"
#include "AStar.hpp"
#include "BoundedVector.hpp"
#include "Message.hpp"
#include "MessageHandler.hpp"
#include "Observer.hpp"
#include "Point.hpp"
#include "Size.hpp"
#include "Region.hpp"
#include <boost/algorithm/string.hpp>

namespace Messaging
{
	class Message;
}

namespace Model
{
	class Robot;
	typedef std::shared_ptr<Robot> RobotPtr;

	class Goal;
	typedef std::shared_ptr<Goal> GoalPtr;

	class Robot :	public AbstractAgent,
					public Messaging::MessageHandler,
					public Base::Observer
	{
		public:
			/**
			 *
			 */
			Robot();
			/**
			 *
			 */
			Robot( const std::string& aName);
			/**
			 *
			 */
			Robot(	const std::string& aName,
					const Point& aPosition);
			/**
			 *
			 */
			virtual ~Robot();
			/**
			 *
			 */
			std::string getName() const
			{
				return name;
			}
			/**
			 *
			 */
			void setName( const std::string& aName,
						  bool aNotifyObservers = true);
			/**
			 *
			 */
			Size getSize() const;
			/**
			 *
			 */
			void setSize(	const Size& aSize,
							bool aNotifyObservers = true);
			/**
			 *
			 */
			Point getPosition() const
			{
				return position;
			}
			/**
			 *
			 */
			void setPosition(	const Point& aPosition,
								bool aNotifyObservers = true);
			/**
			 *
			 */
			BoundedVector getFront() const;
			/**
			 *
			 */
			void setFront(	const BoundedVector& aVector,
							bool aNotifyObservers = true);
			/**
			 *
			 */
			float getSpeed() const;
			/**
			 *
			 */
			void setSpeed( float aNewSpeed,
						   bool aNotifyObservers = true);
			/**
			 *
			 * @return true if the robot is acting, i.e. either planning or driving
			 */
			bool isActing() const
			{
				return acting;
			}
			int randomNumberBetweenUpToN(int N =100 );
			/**
			 *
			 */
			virtual void startActing();
			/**
			 *
			 */
			virtual void stopActing();
			/**
			 *
			 * @return true if the robot is driving
			 */
			bool isDriving() const
			{
				return driving;
			}
			/**
			 *
			 */
			virtual void startDriving();
			/**
			 *
			 */
			virtual void stopDriving();

			void haltDriving();
			/**
			 *
			 * @return true if the robot is communicating, i.e. listens with an active ServerConnection
			 */
			bool isCommunicating() const
			{
				return communicating;
			}
			/**
			 * Starts a ServerConnection that listens at port 12345 unless given
			 * an other port by specifying a command line argument -local_port=port
			 */
			void startCommunicating();
			/**
			 * Connects to the ServerConnection that listens at port 12345 unless given
			 * an other port by specifying a command line argument -local_port=port
			 * and sends a message with messageType "1" and a body with "stop"
			 *
			 */
			void stopCommunicating();
			/**
			 *
			 */
			Region getRegion() const;
			/**
			 *
			 */
			bool intersects( const Region& aRegion) const;
			/**
			 *
			 */
			Point getFrontLeft() const;
			/**
			 *
			 */
			Point getFrontRight() const;
			/**
			 *
			 */
			Point getBackLeft() const;
			/**
			 *
			 */
			Point getBackRight() const;
			/**
			 * @name Observer functions
			 */
			virtual void negotiate();
			/**
			 * Send other robot back a few units
			 */
			virtual void sendBack();

		    

			virtual void drivingAllowed();
			//@{
			/**
			 * A Notifier will call this function if this Observer will handle the notifications of that
			 * Notifier. It is the responsibility of the Observer to filter any events it is interested in.
			 *
			 */
			virtual void handleNotification();
			//@}
			/**
			 *
			 */
			PathAlgorithm::OpenSet getOpenSet() const
			{
				return astar.getOpenSet();
			}
			/**
			 *
			 */
			PathAlgorithm::Path getPath() const
			{
				return path;
			}
			virtual void BroadcastPostion();
			/**
			 * @name Messaging::MessageHandler functions
			 */
			//@{
			/**
			 * This function is called by a ServerSesssion whenever a message is received. If the request is handled,
			 * any response *must* be set in the Message argument. The message argument is then echoed back to the
			 * requester, probably a ClientSession.
			 *
			 * @see Messaging::RequestHandler::handleRequest( Messaging::Message& aMessage)
			 */
			virtual void handleRequest( Messaging::Message& aMessage);
			/**
			 * This function is called by a ClientSession whenever a response to a previous request is received.
			 *
			 * @see Messaging::ResponseHandler::handleResponse( const Messaging::Message& aMessage)
			 */
			virtual void handleResponse( const Messaging::Message& aMessage);
			//@}
			/**
			 * @name Debug functions
			 */
			//@{
							/**
			 * Returns a serialized description of the object
			 */
			virtual std::string serializeRobotInfo() const;
			/**
			 * Returns a 1-line description of the object
			 */
			virtual std::string asString() const;
			/**
			 * Returns a description of the object with all data of the object usable for debugging 	std::string Robot::serializeRobotInfo() const
			 */
			virtual std::string asDebugString() const;
			//@}

			/**
			 * @name The types of messages a Robot should understand
			 */
			//@{
			enum MessageType
			{
				notSupported,
				SyncRequest = 50,
				SyncResponse,
				EchoRequest,
				EchoResponse,
				EchoLocation,
				NegotiateRequest,
				NegotiateResponse,
				SendBackRequest,
				SendBackResponse,
				DriveRequest,
				DriveResponse,
				StartRequest,
				StartResponse,
				SituationOne,
				SituationTwo,
				SituationThree,
				SituationFour,
				SituationFive,
				SituationSix

			};

			virtual void setSituation(Model::Robot::MessageType situation);
//TODO add getter for these three and make them private again
			Size size;
			Point position;
			BoundedVector front;

		protected:
			/**
			 *
			 */
			void drive();
			/**
			 *
			 */
			void calculateRoute(GoalPtr aGoal);
			/**
			 *
			 */
			bool arrived(GoalPtr aGoal);
			/**
			 *
			 */
			bool collision();
		private:
			void restartDriving();
			void fillWorld(std::string messageBody);
			std::string name;

			float speed;

			GoalPtr goal;
			PathAlgorithm::AStar astar;
			PathAlgorithm::Path path;
			GoalPtr startPosition;

			bool acting;
			bool driving;
			bool communicating;
			bool masterDeterminated = false;
			bool win =  false;
			bool droveBack = false;

			std::thread robotThread;
			mutable std::recursive_mutex robotMutex;
	};
} // namespace Model
#endif // ROBOT_HPP_
