#ifndef MAINFRAMEWINDOW_HPP_
#define MAINFRAMEWINDOW_HPP_

#include "Config.hpp"
#include "Widgets.hpp"

namespace Base
{
	class DebugTraceFunction;
} // namespace BaseBase

namespace View
{
	class RobotWorldCanvas;
}

namespace Application
{
	class LogTextCtrl;

	/**
	 *
	 */
	class MainFrameWindow : public Frame
	{
		public:
			/**
			 *
			 * @param aTitle The title which is shown in the title bar
			 */
			MainFrameWindow( const std::string& aTitle);
			/**
			 *
			 */
			virtual ~MainFrameWindow();
			/**
			 *
			 */
			Base::DebugTraceFunction& getTraceFunction() const;

		protected:
			/**
			 *
			 */
			void initialise();
			/**
			 *
			 */
			MenuBar* initialiseMenuBar();
			/**
			 *
			 */
			Panel* initialiseClientPanel();
			/**
			 *
			 */
			SplitterWindow* initialiseSplitterWindow();
			/**
			 *
			 */
			Panel* initialiseLhsPanel();
			/**
			 *
			 */
			Panel* initialiseRhsPanel();
			/**
			 *
			 */
			Panel* initialiseButtonPanel();

		protected:

		private:
			Panel* clientPanel;
			MenuBar* menuBar;
			SplitterWindow* splitterWindow;

			Panel* lhsPanel;
			View::RobotWorldCanvas* robotWorldCanvas;

			Panel* rhsPanel;
			LogTextCtrl* logTextCtrl;
			Panel* buttonPanel;

			Base::DebugTraceFunction* debugTraceFunction;

			void OnQuit( CommandEvent& anEvent);
			void OnWidgetDebugTraceFunction( CommandEvent& anEvent);
			void OnStdOutDebugTraceFunction( CommandEvent& anEvent);
			void OnAbout( CommandEvent& anEvent);

			void OnStartRobot( CommandEvent& anEvent);
			void OnStartBoth( CommandEvent& anEvent);
			void OnStopRobot( CommandEvent& anEvent);
			void OnSituatie1( CommandEvent& anEvent);
			void OnSituatie2(CommandEvent& anEvent);
			void OnSituatie3( CommandEvent& anEvent);
			void OnSituatie4( CommandEvent& anEvent);
			void OnSituatie5( CommandEvent& anEvent);
			void OnSituatie6( CommandEvent& anEvent);
			void OnPopulate( CommandEvent& anEvent);
			void OnUnpopulate( CommandEvent& anEvent);
			void OnStartListening( CommandEvent& anEvent);
			void OnSendMessage( CommandEvent& anEvent);
			void OnStopListening( CommandEvent& anEvent);
			void OnSyncWorld( CommandEvent& UNUSEDPARAM(anEvent));
			
	};
	//	class MainFrameWindow
} //namespace Application

#endif // MAINFRAMEWINDOW_HPP_
