#include "pathpoint.h"
#include <string>

using std::string;
using std::stringstream;
using std::to_string;

void PathPoint::fromString(std::string str){
    stringstream ss(str);
    ss >> x >> y >> theta >> s >> c >> dc;    
}

std::string PathPoint::stringlify(){
    string out = to_string(x) + " " + to_string(y) + " " + to_string(theta) + " " + to_string(s) + " " + to_string(c) + " " + to_string(dc);
    return out;
}


void PathPoint::toJson(nlohmann::json &j){;
    j["x"] = x;
    j["y"] = y;
    j["theta"] = theta;
    j["s"] = s;
    j["c"] = c;
    j["dc"] = dc;
}
