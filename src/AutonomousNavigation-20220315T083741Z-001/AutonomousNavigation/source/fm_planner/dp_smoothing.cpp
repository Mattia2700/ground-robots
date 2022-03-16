#include "dp_smoothing.hpp"
#include "ClothoidAsyPlot.hh"
#include "timeperf.hpp"

#include <omp.h>
#include <atomic>

	
DPSmoothing::DPSmoothing(Grid<real_type> const * grid = nullptr) : grid(grid) {
  omp_set_num_threads(1);
}

bool
DPSmoothing::optimise(vector<Wayline> const & wl, real_type theta_s, real_type theta_g, vector<Curve> & path) {
  waylines = wl;

  const int N_ANGLES_SIDE = 3;
  const int N_REFINEMENTS = 2;

  angles.clear();
  angles.reserve(waylines.size());
  for (int i=0; i<waylines.size(); ++i) {
    angles.emplace_back(waylines[i].theta-M_PI/2., M_PI/2., N_ANGLES_SIDE);
  }
   angles.front().thetac = theta_s;
   angles.front().npts = 0;
//   angles.back().thetac = theta_g;
//   angles.back().npts = 0;

  // for (int i=1; i<waylines.size()-1; ++i) {
  //   angles[i].addSpecial(waylines[i-1].theta-M_PI/2.);
  //   angles[i].addSpecial(waylines[i+1].theta-M_PI/2.);
  // }

  if (!optimise_inner()) return false;

  for (int i=0; i<N_REFINEMENTS; ++i) {
    for (int j=0; j<vangles.size(); ++j) {
      if (!angles[j].npts) continue;
      real_type thetac = angles[j].theta(vangles[j]);
      real_type L = 3./2*angles[j].L/angles[j].npts;
      angles[j].thetac = thetac;
      angles[j].L = L;
    }
    for (int j=0; j<vwaylines.size(); ++j) {
      if (!waylines[j].npts) continue;
      real_type xc = waylines[j].x(vwaylines[j]);
      real_type yc = waylines[j].y(vwaylines[j]);
      real_type L = 3./2*waylines[j].L/waylines[j].npts;

      // CROP TO PREVIOUS WAYLINE??
      // real_type x0n = xc - L*std::cos(waylines[j].theta);
      // real_type y0n = yc - L*std::sin(waylines[j].theta);
      // real_type x1n = xc + L*std::cos(waylines[j].theta);
      // real_type y1n = yc + L*std::sin(waylines[j].theta);

      // real_type s0 = hypot(waylines[j].xc-x0n, waylines[j].yc-y0n);
      // if (s0>waylines[j].L) {
      //   x0n = waylines[j].xc - waylines[j].L*std::cos(waylines[j].theta);
      //   y0n = waylines[j].yc - waylines[j].L*std::sin(waylines[j].theta);
      //   L -= s0-waylines[j].L;
      // }
      
      // real_type s1 = hypot(waylines[j].xc-x1n, waylines[j].yc-y1n);
      // if (s1>waylines[j].L) {
      //   x1n = waylines[j].xc + waylines[j].L*std::cos(waylines[j].theta);
      //   y1n = waylines[j].yc + waylines[j].L*std::sin(waylines[j].theta);
      //   L -= s1-waylines[j].L;
      // }

      // xc = 0.5*(x0n+x1n);
      // yc = 0.5*(y0n+y1n);

      waylines[j].xc = xc;
      waylines[j].yc = yc;
      waylines[j].L  = L;
      //waylines[j].theta = angles[j].thetac + M_PI/2; // TODO: decide
    }

    if (!optimise_inner()) return false;
  }  

  

  path.clear();
  for (int i=1; i<vangles.size(); ++i) {
    Curve spath;
    computeP2P(waylines[i-1].x(vwaylines[i-1]), waylines[i-1].y(vwaylines[i-1]), angles[i-1].theta(vangles[i-1]), 
               waylines[i].x(vwaylines[i]), waylines[i].y(vwaylines[i]), angles[i].theta(vangles[i]), 
               spath);
    path.push_back(spath);
  }
  
  return true;
}

bool 
DPSmoothing::computeP2P( real_type x0, 
            real_type y0,
            real_type th0,
            real_type x1,
            real_type y1,
            real_type th1,
            Curve & path) const {
  const real_type MIN_RADIUS = 0.1/grid->h;//0.5
  path.build_G1(x0, y0, th0, x1, y1, th1, MIN_RADIUS);
  return true;
}

bool 
DPSmoothing::collision(Curve const & path, real_type & score) const {
  //return false;
  //score = 1e20;

   std::vector<HardwareGlobalInterface::People> prediction;
   double vel;
   HardwareGlobalInterface::getInstance().getPredictionData(prediction,vel);
   RobotStatus::LocalizationData pose;
   #ifdef LOC_REALSENSE
   HardwareGlobalInterface::getInstance().getLocalizationDataRealSense(pose);
   #else
   HardwareGlobalInterface::getInstance().getLocalizationDataKarto(pose);
   #endif
 
   //mm = std::make_unique<MapManager>(240, 0.04, MapManager::RobotGeometry{0.5});

  /*
  for (int i = 0 ; i < prediction.size(); i++){
	for (int j = 0 ; j < prediction[i].Xpred[0].size(); j++){
	   std::cout<< prediction[i].Xpred[0][j] << " " ;

	}
  }
   std::cout << 
std::endl;
*/
  //std::cout << "vel: " << vel <<std::endl;
   double v_des = vel/grid->h;

  score = 0;
  if (!grid) return false;
  real_type L = path.length();
  //double Lreal = path.length()*grid->h;
  //std::cout << "length"<< Lreal << " grid "<< grid->h ; 
  int ncuts = ceil(L/(.3/grid->h));
  //int ncutsreal = ceil(Lreal/(0.25*grid->h));
  real_type dl = L/ncuts;
  for (int i=0; i<=ncuts; ++i) {
  //  for (int i=0; i<=0; ++i){
  //std::cout<< i << std::endl;  
    real_type x, y;
    path.eval(i*dl, x, y);
    //double x_real, y_real;
    //x_real =  y * 0.05 + map->x0;
    //y_real =  x * 0.05; //+ map->y0;
    //if (i==0){
    //   std::cout << "xreal: " <<x_real <<" yreal: "<<y_real <<" posex: "<<pose.x << " posey: "<<pose.y<<std::endl;
    //}
    real_type val;

    //double dl_real = dl*grid->h;
    
    double t = (i*dl/v_des);
    int idx = ceil(t/0.1)+10;  
   
    //std::cout << "dlreal " << dl*grid->h << "t " << t << "idx" << idx << " L " << L*grid->h << std::endl;
    //std::cout << i << " : " << ncuts << std::endl;
    //if (i==0){
    //  std::cout << "x: "<< x_real <<" y: "<< y_real <<std::endl;
    //  std::cout << "posx: " << pose.x << " posy: " << pose.y << std::endl;
    //}
    if (x<0 || x>=grid->cols() || y<0 || y>=grid->rows()) {
      val = 0.;
    } 
    else { 
      for (int j = 0 ; j < prediction.size() ; j++) {
          if (idx  >  prediction[j].Xpred[0].size() ){
              val = (*grid)(y,x);
          } else {
             real_type Xp = prediction[j].Xpred[0][9];//[idx-1];
             real_type Yp = prediction[j].Xpred[1][9];//[idx-1];
             if (std::hypot(Xp-x,Yp-y)*grid->h <= 0.6) {
		   val = 0.011;
             } else {
                   val = (*grid)(y,x);
             }
          }

           score += 1/val;
       }
     }
  }
  return false;
}

bool 
DPSmoothing::
optimise_inner() {
  int n = waylines.size();

  // vangles.clear();
  // vangles.reserve(n);

  // struct Cell {
  //   real_type L;
  //   int sampleNxt;
  //   int wNxt;
  //   Cell(): L(0.) {}
  // };
  
  // std::vector<std::vector<std::vector<Cell>>> cells(n-1);  

  cells.resize(n-1);

  int MAX_IDX = n-1; 
  for (int idx=MAX_IDX; idx>=1; --idx) 
  {   
    cells[idx-1].resize(waylines[idx-1].size()); 

    for (int wid1=0; wid1<waylines[idx-1].size(); ++wid1) {
      cells[idx-1][wid1].resize(angles[idx-1].size());
    
      #pragma omp parallel for
      for (int i=0; i<angles[idx-1].size(); ++i) {
        int bestJ = -1;
        int bestW = -1;
        Cost bestC = Cost::WORST();
        //double bestL = 1e20;
        for (int wid2=0; wid2<waylines[idx].size(); ++wid2) 
        {
          for (int j=0; j<angles[idx].size(); ++j) 
          {
            Curve path;
            bool ok = computeP2P(waylines[idx-1].x(wid1), waylines[idx-1].y(wid1), angles[idx-1].theta(i), 
                                 waylines[idx].x(wid2), waylines[idx].y(wid2), angles[idx].theta(j), path);

            if (!ok) continue;

            // real_type k0, k1;
            // k0 = path.kappaBegin();
            // k1 = path.kappaEnd();
            // real_type km = std::max(abs(k0), abs(k1));
            // km = km < grid->h*1.2 ? 0. : km;
      
            real_type score;
            if (collision(path, score)) continue;


            // const real_type W_L = 1;
            //const real_type W_K = 10/grid->h;
            //real_type curL = W_L*path.length() + W_K*path.thetaTotalVariation();
            //real_type curL = (1+W_K*km)*path.length();
            //real_type curL = path.length();        
            //real_type curL = W_K*km + path.length();
            //real_type curL = 0.2*path.dkappa()*path.dkappa() + (1-0.2)*path.length();r

            real_type energy = path.integralCurvature2();
            real_type W_ENERGY = 0.1;//0.1
            score = W_ENERGY*energy + (1-W_ENERGY)*score;

            Cost curC = { score, 0 }; //curL };

            if (idx<MAX_IDX) 
            {
              curC += cells[idx][wid2][j].cost;
            }
            
            if (curC < bestC) {
              bestC = curC;
              bestW = wid2;
              bestJ = j;
            }
          }
        }  
        cells[idx-1][wid1][i].cost = bestC;
        cells[idx-1][wid1][i].sampleNxt = bestJ;
        cells[idx-1][wid1][i].wNxt = bestW;
      }
    }
  }


  // for (int i=0; i<cells.size(); ++i) {
  //   for (int j=0; j<cells[i].size(); ++j) {
  //     for (int k=0; k<cells[i][j].size(); ++k) {
  //       std::cout << i << " " << j << " " << k << ": ";
  //       std::cout << cells[i][j][k].L << " ";
  //       std::cout << cells[i][j][k].wNxt << " ";
  //       std::cout << cells[i][j][k].sampleNxt << " ";
  //       std::cout << std::endl;
  //     }
  //   }
  // }

  int bestIdx = -1;
  int bestW = -1;
  Cost bestC = Cost::WORST();
  for (int w=0; w<cells[0].size(); ++w) {
    for (int i=0; i<cells[0][w].size(); ++i) {
      //std::cout << cells[0][w][i].L << std::endl;
      if (cells[0][w][i].cost < bestC) {
        bestC = cells[0][w][i].cost;
        bestIdx = i;
        bestW = w;
      }
    }
  }
  if (bestIdx==-1) return false;


  vangles.clear(); vwaylines.clear();
  vangles.push_back(bestIdx);
  vwaylines.push_back(bestW);
  Cost prevC = cells[0][bestW][bestIdx].cost;
  for (int i=0; i<n-1; ++i) 
  {
    //std::cout << (prevL-cells[i][bestW][bestIdx].L) << std::endl;
    prevC = cells[i][bestW][bestIdx].cost;
    int nIdx = cells[i][bestW][bestIdx].sampleNxt;
    int wIdx = cells[i][bestW][bestIdx].wNxt;
    vangles.push_back(nIdx);
    vwaylines.push_back(wIdx);
    bestIdx = nIdx;
    bestW = wIdx;
  }

  
  return true;
}

// int main() {
//   DPSmoothing smoothing;

//   vector<real_type> vx, vy;
//   // vx = { 2.9265642, 2.6734362, 2.5109322, 1.9078122, 1.1859282, 1.9249962, 
//   //   2.8265562, 0.00468420000000025, -2.826567, -1.9437558, -1.1859438, 
//   //   - 1.9062558, -2.501565, -2.6734386, -2.9265642, -2.6187522, -1.1406318, 
//   //   - 0.8968758, -1.4562558, -1.9062558, -0.00468780000000013, 1.9078122, 
//   //   1.4468682, 0.8968722, 1.1406282, 2.6187522, 2.9265642 };
//   // vy = { -1.707808758, -1.707808758, -2.367185958, -2.582810358, -2.582810358, 
//   //   - 1.167184758, 0.915619242, 3.178123242, 0.915619242, -1.150000758, 
//   //   - 2.582810358, -2.582810358, -2.393750358, -1.707808758, -1.707808758, 
//   //   - 3.178123242, -3.178123242, -2.989063158, -0.915616758, 0.925003242, 
//   //   2.953123242, 0.925003242, -0.915616758, -2.989063158, -3.178123242, -3.178123242, -1.707808758 };

//   vx = { 1, 2, 3, 4 };
//   vy = { 1, 6, 4, 2 };

//   vector<Wayline> waylines;
//   for (int i=0; i<vx.size(); ++i) {
//     real_type dx = i<vx.size()-1 ? vx[i+1]-vx[i] : vx[i]-vx[i-1];
//     real_type dy = i<vy.size()-1 ? vy[i+1]-vy[i] : vy[i]-vy[i-1];
//     real_type theta = atan2(dy, dx) + M_PI_2;
//     real_type L = (i==0 || i==vx.size()-1) ? 0. : 1;
//     waylines.push_back({vx[i], vy[i], theta, L, L>0?4:0});
//   }

//   vector<Path> path;
//   smoothing.optimise(waylines, 0, 0, path);

//   G2lib::AsyPlot plot("prova90.asy", false);

//   // int c = 0;
//   for (auto & w: waylines) {
//   //   for (int i=0; i<w.size(); ++i) {
//     plot.dot(w.x(0), w.y(0), "green");
//   }
//   //   ++c;
//   // }

//   real_type L = 0;
//   for (auto const & cc: path) {
//     plot.drawClothoid(cc);
//     L += cc.length();
//     std::cout << "ACTUAL LENGTH: " << cc.length() << std::endl;
//   }

//   std::cout << "LENGTH: " << L << std::endl;

//   return 0;
// }
