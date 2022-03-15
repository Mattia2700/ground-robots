#include "Prediction.hpp"

std::vector<std::vector<double>> prediction(std::vector<std::vector<double>> Xin) {

	// filter for positions 

	for (int i=1; i<9; i=i+4){   
	//double d = std::hypot(Xin[0][i+4]-Xin[0][i],Xin[1][i+4]-Xin[1][i]);
        	Xin[0][i+1] = Xin[0][i] + 0.25*(Xin[0][i+4]-Xin[0][i]);
                Xin[1][i+1] = Xin[1][i] + 0.25*(Xin[1][i+4]-Xin[1][i]);
                Xin[0][i+2] = Xin[0][i] + 0.50*(Xin[0][i+4]-Xin[0][i]);
                Xin[1][i+2] = Xin[1][i] + 0.50*(Xin[1][i+4]-Xin[1][i]);
                Xin[0][i+3] = Xin[0][i] + 0.75*(Xin[0][i+4]-Xin[0][i]);
                Xin[1][i+3] = Xin[1][i] + 0.75*(Xin[1][i+4]-Xin[1][i]);

         	//Xin[1][i] = Xin[1][0] + i/Xin[1].size()*(Xin[1][9]-Xin[1][0]);
        }
         

        //XinFil[1][Xin[1].size()] = Xin[1][Xin[1].size()];
	//Xin = XinFil;
//	Xin[0].erase(Xin[0].end()-3,Xin[0].end());
//        Xin[1].erase(Xin[1].end()-3,Xin[1].end());
 
    double X[4][observe+predict];
    double xSet[observe], ySet[observe];
    double out_f0x, out_f0y;
    
 //   std::vector<double> vx,vy;
  //  vx.push_back(0);
   // vy.push_back(0);
    //for( int i = 1; i <Xin[0].size(); i++){
 

    //} 
    // Observation window
    for (int i=0; i<Xin.size(); i++) {
        std::copy(Xin[i].begin(), Xin[i].end(), X[i]);
    }

    // Prediction window
    for (int it=observe; it<observe+predict; it++) {
        // Network input
        for (int i=it; i<it+observe; i++) {
            xSet[i-it] = X[0][i-observe] - X[0][it-observe]; // relative displacements
            ySet[i-it] = X[1][i-observe] - X[1][it-observe];
        }
               
        // Predict forces
        netSfmForces1<observe>(xSet, ySet, out_f0x, out_f0y);

        // SFM dynamical model
        X[0][it] = X[0][it-1] + dt * X[2][it-1];
        X[1][it] = X[1][it-1] + dt * X[3][it-1];
        X[2][it] = X[2][it-1] + dt * (out_f0x * scale) / mass;
        X[3][it] = X[3][it-1] + dt * (out_f0y * scale) / mass;
    }

   

    std::vector<double> x(std::begin(X[0]), std::end(X[0]));
    std::vector<double> y(std::begin(X[1]), std::end(X[1]));
    //std::vector<double> vx(std::begin(X[2]), std::end(X[2]));
    //std::vector<double> vy(std::begin(X[3]), std::end(X[3]));
   
//    x = { x[0] };
//    y = { y[0] };
    std::vector<std::vector<double>> Xout{x, y};//, vx, vy};    
    return Xout;
}

