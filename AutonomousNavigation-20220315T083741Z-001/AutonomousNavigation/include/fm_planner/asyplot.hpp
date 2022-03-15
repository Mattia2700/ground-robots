#ifndef MYASYPLOT_H
#define MYASYPLOT_H

#include <string>
#include <fstream>
#include <iostream>
#include <cmath>


using std::string;
using std::ofstream;

typedef double real_type;

class AsyPlot {
public:
  AsyPlot( string filename );
  ~AsyPlot();

  void
  dot( real_type x, real_type y, string const & penna="black" ) const;

  void
  drawLine( real_type x0, real_type y0,
            real_type x1, real_type y1,
            string const & penna ) const;

  void
  drawRect( real_type x0, real_type y0,
            real_type x1, real_type y1,
            real_type x2, real_type y2,
            real_type x3, real_type y3,
            string const & penna="black") const;

  void
  drawRect( real_type xc, real_type yc,
            real_type w, real_type h,
            string const & penna ) const;


  private:
    mutable ofstream file;
    string  filename;
    bool openFile();
    bool closeFile();
    void initFile();
    void compileFile();
};

#endif 
