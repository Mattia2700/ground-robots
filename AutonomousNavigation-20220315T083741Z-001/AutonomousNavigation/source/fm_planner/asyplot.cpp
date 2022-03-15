#include "asyplot.hpp"


AsyPlot::AsyPlot( string _filename )
  : filename(_filename)
{
  openFile();
  initFile();
}

AsyPlot::~AsyPlot() {
  if ( closeFile() ) compileFile();
}

void
AsyPlot::compileFile() {
  string cmdComp = "asy -f pdf "+ filename;
  system(cmdComp.c_str());
  string pdfFile = filename.substr(0,filename.find(".asy")) + ".pdf";
  std::cout << pdfFile << std::endl;
  string cmdOpen = "(okular " + pdfFile + " &> /dev/null )&";
  system(cmdOpen.c_str());
}

void
AsyPlot::initFile() {
  file
    << "// File generated automatically from C++ \n\n\n"
    << "import graph;\n"
    << "size(14cm,7cm);\n"
    << "\n\n\n";
}

void
AsyPlot::dot( real_type x, real_type y, string const & penna ) const {
  file << "dot((" << x << "," << y << ")," << penna << ");\n\n";
}

void
AsyPlot::drawRect( real_type x0, real_type y0,
                   real_type x1, real_type y1,
                   real_type x2, real_type y2,
                   real_type x3, real_type y3,
                   string const & penna ) const {
	file
    << "filldraw((" << x0 << "," << y0 << ") -- "
    << '(' << x1 << ',' << y1 << ") -- "
    << '(' << x2 << ',' << y2 << ") -- "
    << '(' << x3 << ',' << y3 << ") -- cycle, "
    << penna << ',' << penna << ");\n\n";
}

void
AsyPlot::drawRect( real_type xc, real_type yc,
                   real_type w, real_type h,
                   string const & penna ) const {
  real_type x0 = xc-0.5*w, x1 = xc+0.5*w, 
            y0 = yc-0.5*h, y1 = yc+0.5*h;
  drawRect(x0, y0, x1, y0, x1, y1, x0, y1, penna);
}

void
AsyPlot::drawLine( real_type x0, real_type y0,
                   real_type x1, real_type y1,
                   string const & penna ) const {
	file
    << "draw((" << x0 << "," << y0 << ") -- "
    << '(' << x1 << ',' << y1 << "), "
    << penna << ");\n\n";
}


bool
AsyPlot::openFile() {
  file.open(filename.c_str());
  return file.is_open();
}

bool
AsyPlot::closeFile() {
  file.close();
  return !file.is_open();
}

