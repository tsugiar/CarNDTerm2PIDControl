#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <thread>
#include <chrono>
#include <string>

// for convenience
using nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

using namespace std::chrono_literals;

// Define PID controller parameter Kp, Ki, and Kd here
double PID_Kp = 0.2;
double PID_Ki = 0.1;
double PID_Kd = 30;
bool IsEnteringParameter = false;
bool IsManualTuning = false;

void UpdatePIDParam()
{
	// Thread function for performing manual tuning. Once, this function is activated
	// IsManualTuning boolean value will latch to true. In this case, twiddle algorithm
	// will be disabled permanently.

	double kp, ki, kd;
	std::string read_val, compare_val;

	compare_val = "enter";
	read_val = "";

	IsManualTuning = true;


	while (true)
	{
		std::cout << "\n Type enter to start inputting PID parameters" << std::endl;
		std::cout.flush();

		while (read_val != compare_val)
		{
			std::cin >> read_val;

		}
		read_val = "";

		IsEnteringParameter = true;
		std::cout << "You are in entering parameter mode, output monitor will be closed \n";
		std::cout << "Kp, Ki, and Kd parameters can be entered in numeric form, for-example \n" << "0.02 0.005 15" << std::endl;
		std::cout << "Default Kp, Ki, and Kd values are :" << std::endl << " Kp = " << PID_Kp << " Ki = " << PID_Ki << " Kd = " << PID_Kd << std::endl;
		std::cout.flush();

		try 
		{
			std::cin >> kp >> ki >> kd;
			std::cout << "You are entering \n" << " Kp = " << kp << " Ki = " << ki << " Kd = " << kd << std::endl;
			PID_Kp = kp;
			PID_Ki = ki;
			PID_Kd = kd;
		}
		catch (const std::exception& e)
		{
			std::cout << "Input format for kp, ki, and kd aren't correct \n";
			std::cout << "Correct format should be in numeric, e.g.   0.02 0.005 15 \n";
			std::cout.flush();
		}

		std::cout.flush();

		IsEnteringParameter = false;

	}


}


int main()
{
  uWS::Hub h;
  PID pid;

  // TODO: Initialize the pid variable.
  pid.Init(PID_Kp, PID_Ki, PID_Kd);

  // Uncomment this part if manual-tuning is turned on
  std::thread worker(UpdatePIDParam);


#ifdef _MSC_VER
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode)
#else
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
#endif 
  
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
		  pid.run_counter++;     // Increment pid run-counter
		  // Accumulate error needed by twiddler
		  pid.twiddle_err = pid.twiddle_err + cte*cte;   // L2-norm error

		  // Update PID parameters

		  if (IsManualTuning)
		  {
			  // Keep updating pid parameter when manual tuning is turned on
			  		  pid.Kp = PID_Kp;
			  		  pid.Ki = PID_Ki;
			  		  pid.Kd = PID_Kd;
		  }

		  pid.UpdateError(cte);
		  steer_value = pid.TotalError();

          // DEBUG

		  if (pid.run_counter % 1000 == 0)
		  {
			  if (IsEnteringParameter == false)
			  {
				  std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
			  }
		  }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

		  if (pid.run_counter % 1000 == 0)
		  {
			  if (IsEnteringParameter == false)
			  {
				  std::cout << msg << std::endl;
			  }
		  }

		  // Auto-tune operation (twiddling)
		  // Turning on twiddle (auto-tuning) when manual tuning is off
		  if (IsManualTuning == false)
		  {
			  // Twiddle algorithm for performing auto-tuning.  Uncomment this line
			  // to activate twiddle.
//			  pid.Twiddle();
		  }
	



#ifdef _MSC_VER
		  ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
#ifdef _MSC_VER
		ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#else
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#endif
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });


#ifdef _MSC_VER
     h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) 
#else
	  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
#endif
  {
    std::cout << "Connected!!!" << std::endl;
  });

#ifdef _MSC_VER
	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length)
#else
	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
#endif
  {
#ifdef _MSC_VER
		ws->close();
#else
    ws.close();
#endif
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;

#ifdef _MSC_VER
  auto host = "127.0.0.1";
  if (h.listen(host,port))
#else
  if (h.listen(port))
#endif
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
