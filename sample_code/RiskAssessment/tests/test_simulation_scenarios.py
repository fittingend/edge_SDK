#!/usr/bin/env python3
"""Compile actual scenario functions with middleware-only stubs; no AUTOSAR SDK needed."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1] / "src"

def function(text, name, result="void", start=0):
    first = text.index(result + " " + name + "(", start)
    i = text.index("{", first) + 1
    depth = 1
    while depth:
        depth += (text[i] == "{") - (text[i] == "}")
        i += 1
    return text[first:i]

STUB = r'''#include <vector>
#include <cassert>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
namespace adcm {
struct obstacleListStruct {unsigned obstacle_id=0; int obstacle_class=0; double fused_position_x=0,fused_position_y=0; int stop_count=0; double fused_cuboid_z=0; std::uint64_t timestamp=0;};
struct vehicleListStruct {double position_x=0,position_y=0;};
struct riskAssessmentStruct {unsigned obstacle_id=0; int hazard_class=0; double confidence=0;};
struct risk_assessment_Objects {std::vector<riskAssessmentStruct> riskAssessmentList;};
struct Sink {template<class T> Sink& operator<<(const T&){return *this;}};
struct Log {static Sink Info(){return {};} static Sink Error(){return {};}};
}
using obstacleListVector=std::vector<adcm::obstacleListStruct>;
enum class ObstacleClass {PEDESTRIAN=20};
enum {SCENARIO_1=1,SCENARIO_2,SCENARIO_3,SCENARIO_4,SCENARIO_5,SCENARIO_6,SCENARIO_7,SCENARIO_8,SCENARIO_9,SCENARIO_10};
int gStopValue=1;
const double HEIGHT_THRESH_M=2.0;
#define SCENARIO_LOG_INFO() adcm::Log::Info()
double clampValue(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
double distanceObsToPointDm(const adcm::obstacleListStruct& o,double x,double y){return std::hypot(o.fused_position_x-x,o.fused_position_y-y);}
double distanceEgoToPointDm(const adcm::vehicleListStruct& o,double x,double y){return std::hypot(o.position_x-x,o.position_y-y);}
'''

TESTS = r'''
int main() {
 const std::vector<double> x{0,1000},y{0,0};
 adcm::vehicleListStruct ego{100,0};
 adcm::obstacleListStruct o{};o.obstacle_id=100;o.obstacle_class=20;o.stop_count=25;o.fused_position_x=200;
 auto eligible=[&](double px,bool active=false){o.fused_position_x=px;return isDrivingPathCandidate(o,ego,x,y,300,150,1,active);};
 assert(eligible(200));assert(!eligible(99));assert(eligible(99,true));
 assert(eligible(1,true));assert(!eligible(0,true));assert(!eligible(401));
 ego.position_x=0;assert(!eligible(-50));
 ego.position_x=1050;assert(!eligible(1000));assert(eligible(1000,true));
 ego.position_x=1100;assert(!eligible(1000,true));
 ego.position_x=0;
 // S1 result history: new rear objects excluded; emitted front objects retained
 // until passed by 10m. A MOVE exit must erase this history.
 adcm::risk_assessment_Objects out;
 auto s1=[&](double ox,double ex,int state=3){out.riskAssessmentList.clear();o.fused_position_x=ox;ego.position_x=ex;evaluateScenario1({o},ego,x,y,out,state);return out.riskAssessmentList.size();};
 assert(s1(-50,0)==0);assert(s1(200,100)==1);assert(s1(200,201)==1);
 assert(s1(200,299)==1);assert(s1(200,300)==0);
 assert(s1(200,100)==1);assert(s1(200,100,4)==0);assert(s1(200,201)==0);
 // S3 uses measured displacement, not a velocity field or repeated processing.
 o.obstacle_id=200;o.obstacle_class=1;o.fused_position_y=0;ego.position_x=0;
 auto s3=[&](double ox,std::uint64_t ts,int state=3){out.riskAssessmentList.clear();o.fused_position_x=ox;o.timestamp=ts;evaluateScenario3({o},ego,x,y,out,state);return out.riskAssessmentList.empty()?-1.0:out.riskAssessmentList[0].confidence;};
 assert(s3(100,1000)<0);
 for(int i=2;i<30;i++)assert(s3(100,i*1000)<0); // stationary never ramps
 assert(s3(102,30000)==0.4);
 for(int i=0;i<30;i++)assert(s3(102,30000)==0.4); // duplicate timestamp
 for(int i=1;i<=19;i++)s3(102+2*i,30000+1000*i);
 assert(out.riskAssessmentList.size()==1 && out.riskAssessmentList[0].confidence==1.0);
 assert(s3(140,50000)<0); // stops moving
 assert(s3(142,51000)==0.4); // restarts with a fresh count
 assert(s3(144,1000)<0); // timestamp rollback resets
 assert(s3(146,2000)==0.4);
 assert(s3(146,2000,4)<0);assert(s3(148,3000)<0); // MOVE restart needs observation
 out.riskAssessmentList.clear();evaluateScenario3({},ego,x,y,out,3);
 assert(s3(150,4000)<0); // disappeared/reused ID has no retained confidence
 assert(s3(152,0)<0); // missing timestamp
 std::cout << "PASS: endpoint projection, activation/release, motion, stationary, duplicate timestamps, stop/restart, state reset and ID disappearance\n";
}
'''

def main():
    scenarios = (ROOT / "RiskScenarios.cpp").read_text()
    utils = (ROOT / "RiskAssessmentUtils.cpp").read_text()
    parts = [STUB, function(utils, "calculateDistance", "double"),
             function(utils, "calculateDistance", "double", utils.index("double calculateDistance(") + 1),
             function(utils, "calculateMinDistanceToPath", "bool")]
    for name in ("projectPointToPathArcLengthDm", "isEgoPassedObstacleByPathDm"):
        parts.append(function(scenarios, name, "bool"))
    parts.append(scenarios[scenarios.index("class DrivingScenarioHistory"):scenarios.index("// Coordinate units")])
    parts.append(function(scenarios, "isDrivingPathCandidate", "bool"))
    for number in (1, 2, 3, 5, 6, 9, 10):
        parts.append(function(scenarios, "evaluateScenario" + str(number)))
    parts.append(TESTS)
    with tempfile.TemporaryDirectory(prefix="rass-simul-test-") as directory:
        cpp = Path(directory) / "test.cpp"
        binary = Path(directory) / "test"
        cpp.write_text("\n".join(parts))
        subprocess.run(["g++", "-std=c++20", "-O2", str(cpp), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)

if __name__ == "__main__":
    main()
