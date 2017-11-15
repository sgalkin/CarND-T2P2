#include <iostream>
#include <memory>

#include <uWS/uWS.h>

#include "ProgramOptions.hxx"

#include "application.h"
#include "protocol.h"
#include "laser.h"
#include "radar.h"
#include "filter.h"

namespace {
  po::parser parser() {
    po::parser parser;
    parser["help"].abbreviation('?').description("print this help screen")
      .callback([&parser]{ std::cout << parser << '\n'; });
    parser["radar"].abbreviation('R').type(po::void_).description("use radar measurement");
    parser["laser"].abbreviation('L').type(po::void_).description("use laser measurement");
    return parser;
  }

  MeasurementFilter filter(po::parser parser) {
    MeasurementFilter f;
    auto has_radar = parser["radar"].available();
    auto has_laser = parser["laser"].available();
    
    if(has_radar) f.add(Radar::Tag);
    if(has_laser) f.add(Laser::Tag);
    if(!(has_radar || has_laser)) f.add(Radar::Tag).add(Laser::Tag);

    return f;
  }
}

int main(int argc, char** argv)
{
  auto parser = ::parser();
  if(!parser(argc, argv)) {
    std::cout << parser << "\n";
    return -1;
  }
  auto filter = ::filter(std::move(parser));
  
  uWS::Hub h;
  std::unique_ptr<Application<WSProtocol>> app;

  h.onMessage([&app](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode) {
    try {
      auto message = std::string(data, length);
      auto response = app->ProcessMessage(std::move(message));
      ws.send(response.data(), response.length(), uWS::OpCode::TEXT);
    } catch(std::runtime_error& e) {
      std::cerr << "Error while processing message: " << e.what() << std::endl;
    }
  });

  h.onConnection([&h,&app,&filter](uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest) {
    std::cout << "Connected" << std::endl;
    app.reset(new Application<WSProtocol>(filter));
  });

  h.onDisconnection([&h,&app](uWS::WebSocket<uWS::SERVER>, int, char *, size_t) {
    std::cout << "Disconnected" << std::endl;
    app.reset();
  });

  constexpr int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << port << std::endl;
    return -1;
  }
  h.run();
  return 0;
}
