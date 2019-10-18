// qt5 based visualization application using core.
#include <core/SurfelMapping.h>
#include <glow/glbase.h>
#include "VisualizerWindow.h"

#include <rv/FileUtil.h>
#include <rv/PrimitiveParameters.h>
#include <rv/Stopwatch.h>

#include <locale>

using namespace rv;

int main(int argc, char** argv) {
  QApplication app(argc, argv);

  setlocale(LC_NUMERIC, "C");

  VisualizerWindow window(argc, argv);  // generates the OpenGL context...

  // initialize Laser Fusion.
  rv::ParameterList params;  // default parameters.
  if (argc <= 1) {
    parseXmlFile("../config/default.xml", params);
  } else {
    parseXmlFile(argv[1], params);
  }

  std::shared_ptr<SurfelMapping> fusion = std::shared_ptr<SurfelMapping>(new SurfelMapping(params));

  window.initialize(fusion, params);

  window.show();

  // open file:
  if (argc > 2) {
    std::cout << "Opening " << argv[2] << std::endl;
    window.openFile(QString(argv[2]));
  }

  int32_t ret = app.exec();

  std::cout << rv::Stopwatch::active() << " stopwatches active at exit." << std::endl;

  return ret;
}
