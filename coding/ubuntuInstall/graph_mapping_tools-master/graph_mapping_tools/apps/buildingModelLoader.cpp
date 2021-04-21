#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <iostream>
//#include "Eigen/Eigen/Eigen"

#include <Eigen/Dense>
#include<Eigen/Core>
// fstream
#include <fstream>
#include<sstream>
#include <stdlib.h>
#include <iomanip>

// math
#include <math.h>
//time 
#include <time.h>
//algorithm 
#include <algorithm>
// Define Infinite (Using INT_MAX caused overflow problems)
#define INF 10000
using namespace Eigen;

using namespace std;

class buildingModelLoader
{
  ros::NodeHandle nh_;

public:

  typedef struct
  {
    float x;
    float y;
    float azimuth;
    float maskElevation;
  }floatPoint; // pixel point in image 

  typedef struct
  {
    float x;
    float y;
    float z;
    float ID;
  }bp; // building point 



public:
  buildingModelLoader(string kml_dir, string saved_skymask_dir,bool mannually_set_ref_llh, double ref_lat,double ref_lon, 
    double ref_alt, double ENU_LiDAR_Angle)
  {
    std::cout<<"this is the building model loading node..."<<std::endl;
    pub_debug_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/HDMap/buildingModels", 1, true);
    marker_pub = nh_.advertise<visualization_msgs::Marker>("/markerDebugger", 10);

    mannually_set_ref_llh_ = mannually_set_ref_llh;
    ref_lat_fromlaunch = ref_lat;
    ref_lon_fromlaunch = ref_lon;
    ref_alt_fromlaunch = ref_alt;
    ref_ENU_LiDAR_Angle_fromlaunch = ENU_LiDAR_Angle;
    //readKml("G:/Dropbox/Dropbox/PNL/57HuaWei_WWS/Shadow_Matching_Huawei/Shadow_Matching_Huawei/NiuToujiao.kml"); // demo_HK_TST_fix
    // readKml("/home/wenws/amsipolyu/src/rtklibros/app/generate_skymask/data/demo_HK_TST_fix.kml"); // demo_HK_TST_fix
    readKml(kml_dir);
    wait(1);
    string2llh(builDataS); // parse llh from string
    wait(1);
    prepareBuildingData(buildingLLHS);  // prepare buildingS to map_
    wait(1);


    Visu3DBuildingModels();  
  }

  ~buildingModelLoader()
  {
  }


  void Visu3DBuildingModels()
  {
    std::cout<<"visualization..."<<std::endl;
    float f =0;
    // map_.buildingS_M
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/map";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.5;
    points.scale.y = 3;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 1.5;
    line_strip.scale.y = 1.5;
    line_strip.scale.z = 1.5;
    line_list.scale.x = 0.5;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 100; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }

    for(int i =0; i<map_.buildingS_M.size();i++)
    {
      for(int j=0; j<map_.buildingS_M[i].buildingENUV.size()-1;j++)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        // marker.header.stamp = filtered->header.stamp;
        marker.ns = "Rays";
        marker.id = markers.markers.size();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = 0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.6; // 0.1
        marker.points.resize(2);
        marker.points[0].x = map_.buildingS_M[i].buildingENUV[j].E;
        marker.points[0].y = map_.buildingS_M[i].buildingENUV[j].N;
        marker.points[0].z = map_.buildingS_M[i].buildingENUV[j].U - map_.llhOri(2);
        marker.points[1].x = map_.buildingS_M[i].buildingENUV[j+1].E;
        marker.points[1].y = map_.buildingS_M[i].buildingENUV[j+1].N;
        marker.points[1].z = map_.buildingS_M[i].buildingENUV[j+1].U - map_.llhOri(2);
        marker.colors.resize(2);
        marker.colors[0].a = 1;
        marker.colors[0].r = 0.0;
        marker.colors[0].g = 1.0;
        marker.colors[0].b = 0.0;
        marker.colors[1].a = 1;
        marker.colors[1].r = 0.0;
        marker.colors[1].g = 1.0;
        marker.colors[1].b = 0.0;

        markers.markers.push_back(marker);
      }
      std::cout<<"building number: "<<map_.buildingS_M.size()<<"  number of edge points in buildings   "<<map_.buildingS_M[i].buildingENUV.size()<<std::endl;
    }

    for(int i =0; i<map_.buildingS_M.size();i++)
    {
      for(int j=0; j<map_.buildingS_M[i].buildingENUV.size()-1;j++)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        // marker.header.stamp = filtered->header.stamp;
        marker.ns = "Rays";
        marker.id = markers.markers.size();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = 0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.6; // 0.1
        marker.points.resize(2);
        marker.points[0].x = map_.buildingS_M[i].buildingENUV[j].E;
        marker.points[0].y = map_.buildingS_M[i].buildingENUV[j].N;
        marker.points[0].z = 0;
        marker.points[1].x = map_.buildingS_M[i].buildingENUV[j+1].E;
        marker.points[1].y = map_.buildingS_M[i].buildingENUV[j+1].N;
        marker.points[1].z = 0;
        marker.colors.resize(2);
        marker.colors[0].a = 1;
        marker.colors[0].r = 0.0;
        marker.colors[0].g = 1.0;
        marker.colors[0].b = 0.0;
        marker.colors[1].a = 1;
        marker.colors[1].r = 0.0;
        marker.colors[1].g = 1.0;
        marker.colors[1].b = 0.0;

        markers.markers.push_back(marker);
      }
      std::cout<<"building number: "<<map_.buildingS_M.size()<<"  number of edge points in buildings   "<<map_.buildingS_M[i].buildingENUV.size()<<std::endl;
    }

    for(int i =0; i<map_.buildingS_M.size();i++)
    {
      for(int j=0; j<map_.buildingS_M[i].buildingENUV.size()-1;j++)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        // marker.header.stamp = filtered->header.stamp;
        marker.ns = "Rays";
        marker.id = markers.markers.size();
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = 0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.6; // 0.1
        marker.points.resize(2);
        marker.points[0].x = map_.buildingS_M[i].buildingENUV[j].E;
        marker.points[0].y = map_.buildingS_M[i].buildingENUV[j].N;
        marker.points[0].z = map_.buildingS_M[i].buildingENUV[j].U - map_.llhOri(2);;
        marker.points[1].x = map_.buildingS_M[i].buildingENUV[j].E;
        marker.points[1].y = map_.buildingS_M[i].buildingENUV[j].N;
        marker.points[1].z = 0;
        marker.colors.resize(2);
        marker.colors[0].a = 1;
        marker.colors[0].r = 0.0;
        marker.colors[0].g = 0.0;
        marker.colors[0].b = 1.0;
        marker.colors[1].a = 1;
        marker.colors[1].r = 0.0;
        marker.colors[1].g = 0.0;
        marker.colors[1].b = 1.0;

        markers.markers.push_back(marker);
      }
      std::cout<<"building number: "<<map_.buildingS_M.size()<<"  number of edge points in buildings   "<<map_.buildingS_M[i].buildingENUV.size()<<std::endl;
    }

    // Draw the uncertainty ellipse
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "ellipse";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 10; // /10
    marker.scale.y = 10; // /10
    marker.scale.z = 10; // /10
    marker.color.a = 0.99; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markers.markers.push_back(marker);



    // marker_pub.publish(points);
    marker_pub.publish(line_strip);
    // markers.markers.push_back(line_strip);
    pub_debug_marker_.publish(markers);

    // marker_pub.publish(line_list);
    

  }



  /**
   * @brief delay function
   * @param seconds for delay
   * @return void
   @ 
   */
  void wait(int seconds) // delay function
  {
    clock_t endwait,start;
    start = clock();
    endwait = clock() + seconds * CLOCKS_PER_SEC;
    while (clock() < endwait) {
      if(clock() - start > CLOCKS_PER_SEC)
      {
        start = clock();
        std::cout<<".......1 s"<<std::endl;
      }
    }
  }

public: // solve convex hull problem (https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#C++)
    // Implementation of Andrew's monotone chain 2D convex hull algorithm.
    // Asymptotic complexity: O(n log n).
    // Practical performance: 0.5-1.0 seconds for n=1000000 on a 1GHz computer.
  typedef double coord_t;         // coordinate type
  typedef double coord2_t;  // must be big enough to hold 2*max(|coordinate|)^2

  struct Point {
    coord_t x, y;

    bool operator <(const Point &p) const {
      return x < p.x || (x == p.x && y < p.y);
    }
  };

  // 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
  // Returns a positive value, if OAB makes a counter-clockwise turn,
  // negative for clockwise turn, and zero if the points are collinear.
  coord2_t cross(const Point &O, const Point &A, const Point &B)
  {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
  }

  // Returns a list of points on the convex hull in counter-clockwise order.
  // Note: the last point in the returned list is the same as the first one.
  vector<Point> convex_hull(vector<Point> P)
  {
    size_t n = P.size(), k = 0;
    if (n <= 3) return P;
    vector<Point> H(2 * n);

    // Sort points lexicographically
    sort(P.begin(), P.end());

    // Build lower hull
    for (size_t i = 0; i < n; ++i) {
      while (k >= 2 && cross(H[k - 2], H[k - 1], P[i]) <= 0) k--;
      H[k++] = P[i];
    }

    // Build upper hull
    for (size_t i = n - 1, t = k + 1; i > 0; --i) {
      while (k >= t && cross(H[k - 2], H[k - 1], P[i - 1]) <= 0) k--;
      H[k++] = P[i - 1];
    }

    H.resize(k - 1);
    return H;
  }

public: // solve the problem: if one point is inside the building
  struct Point_
  {
    double x;
    double y;
  };

  // Given three colinear Point_s p, q, r, the function checks if
  // Point_ q lies on line segment 'pr'
  bool onSegment(Point_ p, Point_ q, Point_ r)
  {
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
      q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
      return true;
    return false;
  }

  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  int orientation(Point_ p, Point_ q, Point_ r)
  {
    int val = (q.y - p.y) * (r.x - q.x) -
      (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0; // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
  }

  // The function that returns true if line segment 'p1q1'
  // and 'p2q2' intersect.
  bool doIntersect(Point_ p1, Point_ q1, Point_ p2, Point_ q2)
  {
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
      return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
  }

  // Returns true if the Point_ p lies inside the polygon[] with n vertices
  bool isInside(vector<Point_> polygon, int n, Point_ p)
  {
    // There must be at least 3 vertices in polygon[]
    if (n < 3) return false;

    // Create a Point_ for line segment from p to infinite
    Point_ extreme = { INF, p.y };

    // Count intersections of the above line with sides of polygon
    int count = 0, i = 0;
    do
    {
      int next = (i + 1) % n;

      // Check if the line segment from 'p' to 'extreme' intersects
      // with the line segment from 'polygon[i]' to 'polygon[next]'
      if (doIntersect(polygon[i], polygon[next], p, extreme))
      {
        // If the Point_ 'p' is colinear with line segment 'i-next',
        // then check if it lies on segment. If it lies, return true,
        // otherwise false
        if (orientation(polygon[i], p, polygon[next]) == 0)
          return onSegment(polygon[i], p, polygon[next]);

        count++;
      }
      i = next;
    } while (i != 0);

    // Return true if count is odd, false otherwise
    return count & 1; // Same as (count%2 == 1)
  }


public: // variables for particles
  double reserve;
  typedef struct obs_Nav_epoch // reserved
  {

  };

  typedef struct  // satellite information
  {
    double GNSS_time;
    double total_sv; // total satellite in this epoch
    double prn_satellites_index; // satellite prn 
    double pseudorange; // satellite pseudorange
    double snr; // satellite snr
    double elevation; // satellite elevation 
    double azimuth; // satellite azimuth 
    double err_tropo; //satellite erro_tropo
    double err_iono; // satellite ono
    double sat_clk_err; // satellite clock bias 
    double sat_pos_x; // satellite position x
    double sat_pos_y; // satellite position y
    double sat_pos_z; // satellite position z
    int visable; // satellite visability
    std::string sat_system; // satellite system ("GPS", "GLONASS")
  }satelliteInfo;

  typedef struct // single grid: determined in ENU coordiante system
  {
    double E_;
    double N_;
    double U_;
    double score;
  }grid;

  typedef struct  //  grid sequences (grids)
  {
    std::vector<grid> grids_;
  }grids;

  typedef struct  // state for a particle
  {
    double GNSS_time = 0; // time stamps
    //satellites information 
    std::vector<satelliteInfo>  satInfo; // incluide pseudorange.etc
    int particle_ID; // the ID for particle
    double scores; // scores for this particle 
    //position in llh
    double lon; // latitude
    double lat; // longtutude
    double altitude; // altitude

  
    grids smGrids;  //grid at present for shadow matching (plane)
    std::vector<grids> smGridss; // all the grids from first epoch to last epoch (multi-plane)

    //position in ENU
    double E;
    double N;
    double U;
    double ini_lon;
    double ini_lat;
    double ini_alt;

  }particle;
  std::vector<particle> particles; // all th particles at present
  std::vector<std::vector<particle>> particless; // all the particles from first epoch to last epoch
public: // for reading data from kml file
  vector<string> builDataS; // building data in string format
  typedef struct 
  {
    double lon;
    double lat;
    double alt;
  }LLH;
  vector <LLH> buildingLLH; // determine a building with a set of lon lat alt
  vector<vector <LLH>> buildingLLHS; // all the buildings

public:  // for generate map
  typedef struct
  {
    double E;
    double N;
    double U;
  }ENU;
  typedef struct // struct for a processed building information
  {
    double center_E; // center position of the building in E direction
    double center_N; // center position of the building in N direction
    double center_U; // center position of the building in U direction
    double ini_lon; // initial reference position for ENU
    double ini_lat; // initial reference position for ENU
    double ini_alt; // initial reference position for ENU
    vector <LLH> buildingLLHV; // building node list (each node with respect to a vertex) in llh
    vector <ENU> buildingENUV; // building node list (each node with respect to a vertex) in ENU
    vector<Point> buildingHull; // building hull points (used for later check if the point is inside the building) in ENU
    double sAzimuth; // smallest azimuth
    double bAzimuth; // bigest azimuth


  } building;
  vector<building> buildingS; // processed building information
  typedef struct  // struct a map
  {
    vector<building> buildingS_M; // processed building information save in a map struct
    // lon lat alt boundary 
    double lonMax; // max longitude
    double latMax; // max latitude
    double altMax; // max altitude
    MatrixXd llhMax = MatrixXd::Random(3, 1); // available 

    double lonMin; // min longitude
    double latMin; // min latitude
    double altMin; // min altitude
    MatrixXd llhMin = MatrixXd::Random(3, 1); // available 

    // ENU boundary
    double EMax; // max E
    double NMax; // max N
    double UMax; // max U
    MatrixXd ENUMax = MatrixXd::Random(3, 1); // available 

    double EMin; //  min E
    double NMin; //  min N
    double UMin; //  min U
    MatrixXd ENUMin = MatrixXd::Random(3, 1); // available 

    // original llh
    double lonOri; // original lon of ENU (refers to reference point)
    double latOri; // original lat of ENU (refers to reference point)
    double altOri; // original alt of ENU (refers to reference point)
    MatrixXd llhOri = MatrixXd::Random(3, 1);

    // minimum elevation angle and corresponding maxinum searching distance
    double MinElevation = 15; // satellites with a elevation less than 15 degree will not be considered
    double maxSearDis; // the maximum searching distance when finding the insections
    // azimuth search resolution 
    double aziReso = 1.0; // azimuth search resolution 
    double disReso = 2.0; // grid resolution for skymasks
  }map;
  map map_; // the map saving all the buildings

public:  // for generate sky mask
  typedef struct // mask elevation
  {
    double azimuth; // the azimuth 
    double elevation; // the mask elevation
  }elevationMask;

  typedef struct
  {
    LLH posLLH; // pose in LLH
    ENU poseENU; // pose in ENU
    double buildingNum; // how many this position can intersect: if point inside building (buildingNum =1)
    bool insideBuilding; // inside buildings (=1) , outside building (=0)
    vector<elevationMask> aziElemask; // a vector 
  }skyMask; // the sky mask in a position (refers to the skyplot in one position)
  vector<skyMask> skyMaskS; // save all the skymsk in one vector 

public: // identify the intersection
  // Function used to display X and Y coordinates
  // of a point
  void displayPoint(Point_ P)
  {
    cout << "(" << P.x << ", " << P.y
      << ")" << endl;
  }

  Point_ lineLineIntersection(Point_ A, Point_ B, Point_ C, Point_ D)
  {
    Point_ result;
    // Line AB represented as a1x + b1y = c1
    double a1 = B.y - A.y;
    double b1 = A.x - B.x;
    double c1 = a1*(A.x) + b1*(A.y);

    // Line CD represented as a2x + b2y = c2
    double a2 = D.y - C.y;
    double b2 = C.x - D.x;
    double c2 = a2*(C.x) + b2*(C.y);

    double determinant = a1*b2 - a2*b1;

    double x = (b2*c1 - b1*c2) / determinant;
    double y = (a1*c2 - a2*c1) / determinant;
    result.x = x;
    result.y = y;
    return result;
  }
  
  double disPP(Point_ A, Point_ B)
  {
    return (double ) sqrt(((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y)));
  }

public: // functions 
    /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: read kml and extract one building lla into a string
  input: kml file
  output: vector<string> variable
  */
  void readKml(string filepath) // read kml file 
  {
    /* test code*/
    /*ifstream OpenFile("NiuToujiao.kml");
    char ch;
    while (!openfile.eof())
    {
      openfile.get(ch);
      cout << ch;
    }
    openfile.close();*/
    std::cout << "[Function]->readkml read .kml files to strings from: " <<filepath<<"   "<< std::endl;
    ifstream in(filepath);
    string filename;
    string line; // save content of one line 
    int lineNum = 0; // line number
    string search = "coordinates"; // searching 
    vector<int> cLine; // save line numbers
    vector<string> allLines; // save all the lines of the kml file
    
    if (in) // file is available   
    {
      std::cout<<"begin read kml files..."<<std::endl;
        while (getline(in, line)) // line not contain line break (enter)
        {
            lineNum++;
            //cout << line << endl;
            //std::cout << "line number" << lineNum << endl;
            allLines.push_back(line);
            if (line.find(search, 0) != string::npos) {
              //cout << "found: " << search << "line number : " << lineNum << endl;
              cLine.push_back(lineNum-1); //  save line number
            }
        }
        cout << "building amounts:" << cLine.size()/2 << endl; // amount of buildings
        for (auto it = cLine.begin(); it != cLine.end(); it++) {
          int temp = *it;
          //cout << "one building data: " << allLines[temp+1] << endl;
          builDataS.push_back(allLines[temp + 1]);
          it++;
        }
        for (auto it = builDataS.begin(); it != builDataS.end(); it++) {
          cout << "-----------------------------------buildings--------------------------------" << "\n" << *it << "\n" << endl;
        }

    }
    else // file is not available
    {
      cout << "no such kml file" << endl;
    }
  }

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: parse llh from string 
  input: vector<string>
  output: vector<vector <LLH>> buildingLLHS; // all the buildings
        // the first [] indicates the building index, the second [] indicates the lla index in one building
  */
  void string2llh(vector<string> data)// extract the llh from the string for each building
  {
    std::cout << "[Function]->string2llh llh string to llh double..." << std::endl;
    vector<string> aBuilding;
    for (auto it = data.begin(); it != data.end(); it++) { // for one building
      string temp = *it;

      string result;
      stringstream input(temp); // split by blank
      while (input >> result)
        aBuilding.push_back(result);
      for (int i = 0; i<aBuilding.size(); i++) { // for one building again 
        // cout << "each lon lat alt:  "<<aBuilding[i] << endl;
        // cout << "size of this building :  " << aBuilding.size() << endl;
        //split the string 114.1772621294604,22.29842880200087,58 and save it to  :vector <LLH> buildingLLH; 
        std::stringstream ss(aBuilding[i]); // split into three string
        vector<string> result;
        while (ss.good())
        {
          string substr;
          getline(ss, substr, ',');
          result.push_back(substr);
        }
        LLH llh_;
        for (int k = 0; k < result.size(); k++)
        {
          //std::cout<<"string result: " << result.at(k) << std::endl;
          string word;
          word = result.at(k);
          double value = strtod(word.c_str(), NULL);
          std::cout << std::setprecision(17);
          //std::cout<< "double result: " << value << '\n';
          
          if (k == 0) {
            llh_.lon = value;
          }
          else if (k == 1) llh_.lat = value;
          else if (k == 2) llh_.alt = value;
        }
        buildingLLH.push_back(llh_);  
      }
      aBuilding.clear(); // clear this after process a building (very important)
      cout << "size of llh vector save in one building llh: " << buildingLLH.size() << "\n" << endl;
      cout << "finish one buildings......................................................................................................................." << endl;
      buildingLLHS.push_back(buildingLLH); // each building save in this vector
      buildingLLH.clear(); // clear this after fully finished one building (very important)
      

    }
    cout << "finish all buildings, total building number is : " << buildingLLHS.size()<<"\n" << endl; // 
    std::cout << std::setprecision(17);
    // the first [] indicates the building index, the second [] indicates the lla index in one building
    // cout << std::setprecision(17)<< "test: " << buildingLLHS[0][0].lat - buildingLLHS[0][1].lat << "\n" << endl; 
    cout << "-----------------------------------------------3D city map have been read into vector buildingLLHS ----------------------------------------\n\n\n\n\n\n\n\n" << endl; // 
  }

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: preparing building data
  input:vector<vector <LLH>> data // the first [] indicates the building index, the second [] indicates the lla index in one building
  output:  prepared building information

  Notation:
      first node of the first building will be choose as the initial reference point for ENU
  */
  void prepareBuildingData(vector<vector <LLH>> data) // prepare the building information: do some pre-processing
  {
    
    std::cout << "[Function]->prepareBuildingData transfer builidngs points, convex hull solving..." << std::endl;
    //center of the building


    /* initialize the reference point of the map: two methods
    1. initialize by boundary of the 3D city maps
    2. initialize by the provided ref lat and lon from generateSkymask.launch 
    */
    if(mannually_set_ref_llh_ == false)
    {
      map_.llhOri(0) = data[0][0].lon;
      map_.llhOri(1) = data[0][0].lat;
      map_.llhOri(2) = data[0][0].alt;
    }
    else if(mannually_set_ref_llh_ == true)
    {
      map_.llhOri(0) = ref_lon_fromlaunch; //22.3111737354
      map_.llhOri(1) = ref_lat_fromlaunch;
      map_.llhOri(2) = ref_alt_fromlaunch;
    }

    // specically for data collected in moko of Hong Kong
    map_.llhOri(0) = 114.16931259; //22.3111737354
    map_.llhOri(1) = 22.3111737354;
    map_.llhOri(2) = ref_alt_fromlaunch;
    
    float count = 0; 
    double lonMa=-100000; // lon max 
    double latMa=-100000; // lat max 
    double altMa=-100000; // alt max
    
    double lonMi=1000000; // lon min 
    double latMi=1000000; // lat min
    double altMi=1000000; // alt min

    for (int i = 0; i < data.size() ; i++) // index all the buildings 
    {
      building building_; // processed: a buidling information
      building_.ini_lon = map_.llhOri(0); // original lon 
      building_.ini_lat = map_.llhOri(1); // original lat
      building_.ini_alt = map_.llhOri(2); // original alt
      for (int j = 0; j < data[i].size() ; j++) // index the llh in one buildings
      {
        count++;
        building_.buildingLLHV.push_back(data[i][j]); // save llh in struct
        // save ENU
        //****obtain ecef
        Eigen::MatrixXd llh;
        llh.resize(3, 1);
        Eigen::MatrixXd ecef;
        ecef.resize(3, 1);
        llh(0) = data[i][j].lon;
        llh(1) = data[i][j].lat;
        llh(2) = data[i][j].alt;
        ecef = llh2ecef(llh); // position in ecef
        
        //****obtain ENU
        Eigen::MatrixXd llhO;  //original
        llhO.resize(3, 1);
        //llhO(0) = data[0][0].lon; //original lon for ENU reference 
        //llhO(1) = data[0][0].lat; //original lat for ENU reference 
        //llhO(2) = data[0][0].alt; //original alt for ENU reference 

        llhO(0) = building_.ini_lon; //original lon for ENU reference 
        llhO(1) = building_.ini_lat; //original lat for ENU reference 
        llhO(2) = building_.ini_alt; //original alt for ENU reference 
        Eigen::MatrixXd enu;
        enu.resize(3, 1);
        enu = ecef2enu(llhO, ecef);

        double prex_ = enu(0);
        double prey_ = enu(1);
        // double theta = (ref_ENU_LiDAR_Angle_fromlaunch )*( 3.141592 / 180.0 ); // 68.5
        double origin_azimuth = 348.747632943  - 1.35;
        origin_azimuth = 360 - origin_azimuth;
        double theta = -1 * (origin_azimuth - 90)*( 3.141592 / 180.0 );
        enu(0) = (prex_ * cos(theta) - prey_ * sin(theta)) + 1.3 + 1.3 +1;
        enu(1) = (prex_ * sin(theta) + prey_ * cos(theta)) + 1.3 ; 

        

        //****save ENU into building struct
        ENU ENU_;
        // ENU_.E = enu(1); // save E
        // ENU_.N = -enu(0); // save N
        ENU_.E = enu(0); // save E
        ENU_.N = enu(1); // save N
        ENU_.U = data[i][j].alt; // save height   ENU_.U = data[i][j].alt; // save height
        building_.buildingENUV.push_back(ENU_); // save ENU to 
        //cout << "total llh->count: " << count << "    building number->data.size()" << data.size() << "      llh number->data[i].size()" << data[i].size() << "   ecef:->" << ecef << "    enu:->" << enu << "\n"<<endl;
        
        //obtain the boundary of all the buidlings
        if (llh(0) > lonMa) lonMa = llh(0); // save the largest lon
        if (llh(1) > latMa) latMa = llh(1); // save the largest lat
        if (llh(2) > altMa) altMa = llh(2); // save the largest alt

        if (llh(0) < lonMi) lonMi = llh(0); // save the smallest lon
        if (llh(1) < latMi) latMi = llh(1); // save the smallest lat
        if (llh(2) < altMi) altMi = llh(2); // save the smallest alt

      }

      // solve the convex hull probelm, extract the hull of the building in 2D domain (in ENU)
      vector<Point> buildingNodePointFormat;
      for (int index = 0; index < building_.buildingENUV.size(); index++) // index all the buildings 
      {
        Point temp_;
        temp_.x = building_.buildingENUV[index].E;
        temp_.y = building_.buildingENUV[index].N;
        buildingNodePointFormat.push_back(temp_); // save all the node (ENU) into the Point format
        //cout << "temp_x:  " << temp_.x<< "     temp_y:  " << temp_.y<<"\n";
      }
      building_.buildingHull = convex_hull(buildingNodePointFormat);
      // cout << "before convex hull : " << buildingNodePointFormat.size() << "      after convex hull: " << building_.buildingHull.size()<<"\n";
      // cout << "enu size -------------------------------------------- ->: " << building_.buildingENUV.size() << "\n\n\n\n\n\n\n\n\n";
      buildingS.push_back(building_);
    }

    map_.buildingS_M = buildingS; // save all the buildings into the map 
    //save the boundary lon lat alt
    map_.lonMax = lonMa;
    map_.latMax = latMa;
    map_.altMax = altMa;
    map_.llhMax(0) = lonMa;
    map_.llhMax(1) = latMa;
    map_.llhMax(2) = altMa;

    map_.maxSearDis = map_.llhMax(2) / tan(15 * (3.1415926 / 180.0)); // calculate the maximum searching distance 

    map_.lonMin = lonMi;
    map_.latMin = latMi;
    map_.altMin = altMi;
    map_.llhMin(0) = lonMi;
    map_.llhMin(1) = latMi;
    map_.llhMin(2) = altMi;

    
    
    //cout << "original point " << map_.llhOri << "\n";
    //save the boundary in ENU : boundary means the maximum and minmum
    Eigen::MatrixXd ecefTemp; // save the max 
    ecefTemp.resize(3, 1);
    ecefTemp = llh2ecef(map_.llhMax); // position in ecef
    map_.ENUMax = ecef2enu(map_.llhOri, ecefTemp);
    cout << "map enu max   " << map_.ENUMax<<"  map_.llhOri-> "<<map_.llhOri<<"\n";

    ecefTemp.resize(3, 1); // save the min 
    ecefTemp = llh2ecef(map_.llhMin); // position in ecef
    map_.ENUMin = ecef2enu(map_.llhOri, ecefTemp);
    cout << "map enu min " << map_.ENUMin << "\n";

    cout << "-----------map information--------" << map_.buildingS_M.size() << endl;
    /*
    map_.lonMax :-122.3002392264451      map_.latMax 37.901411108972418      
    map_.lonMin :-122.302402266828      map_.latMin 37.900422296017631
    */
    // cout << "map_.lonMax :" << map_.lonMax << "      map_.latMax " << map_.latMax << "      map_.lonMin :" << map_.lonMin << "      map_.latMin " << map_.latMin<<"\n\n";

    cout << "-------------------------------------------------finish preparing all the buildings information and is saved to a map struct: ->  map_------------------------------------------------\n\n";
  }


  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: building to marker in ROS
  input: building building_; // processed: a buidling information
  output: 
  */
  visualization_msgs::MarkerArray buildingENU2Marker(building building_) // 
  {
    
  }

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: llh to ecef
  input: llh (Matrix3d)
  output: ecef (Matrix3d)
  */
  Eigen::MatrixXd llh2ecef(Eigen::MatrixXd data) // transform the llh to ecef
  {
    Eigen::MatrixXd ecef; // the ecef for output
    ecef.resize(3, 1);
    double a = 6378137.0;
    double b = 6356752.314;
    double n, Rx, Ry, Rz;
    double lon = (double)data(0) * 3.1415926 / 180.0; // lon to radis
    double lat  = (double)data(1) * 3.1415926 / 180.0; // lat to radis
    double alt  = (double)data(2); // altitude
    n = a * a / sqrt(a * a * cos(lat) * cos(lat) + b * b * sin(lat) * sin(lat));
    Rx = (n + alt) * cos(lat) * cos(lon);
    Ry = (n + alt) * cos(lat) * sin(lon);
    Rz = (b * b / (a * a) * n + alt) * sin(lat);
    ecef(0) = Rx; // return value in ecef
    ecef(1) = Ry; // return value in ecef
    ecef(2) = Rz; // return value in ecef
    return ecef;

    /**************for test purpose*************************
    Eigen::MatrixXd llh;
    llh.resize(3, 1);
    Eigen::MatrixXd ecef;
    ecef.resize(3, 1);
    llh(0) = 114.1772621294604;
    llh(1) = 22.29842880200087;
    llh(2) = 58;
    ecef = llh2ecef(llh);
    cout << "ecef ->: " << ecef << "\n";
    */
  }

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: ecef to llh
  input: ecef (Matrix3d)
  output: llh (Matrix3d)
  */
  Eigen::MatrixXd ecef2llh(Eigen::MatrixXd data) // transform the ecef to llh
  {
    Eigen::MatrixXd llh; // the ecef for output
    double pi = 3.1415926; // pi
    llh.resize(3, 1);
    double x = data(0); // obtain ecef 
    double y = data(1);
    double z = data(2);
    double x2 = pow(x, 2);
    double y2 = pow(y, 2);
    double z2 = pow(z, 2);

    double a = 6378137.0000; //earth radius in meters
    double b = 6356752.3142; // earth semiminor in meters
    double e = sqrt(1 - (b / a) * (b / a));
    double b2 = b*b;
    double e2 = e*e;
    double  ep = e*(a / b);
    double  r = sqrt(x2 + y2);
    double  r2 = r*r;
    double  E2 = a * a - b*b;
    double F = 54 * b2*z2;
    double G = r2 + (1 - e2)*z2 - e2*E2;
    double c = (e2*e2*F*r2) / (G*G*G);
    double s = (1 + c + sqrt(c*c + 2 * c));
    s = pow(s, 1 / 3);
    double P = F / (3 * ((s + 1 / s + 1)*(s + 1 / s + 1)) * G*G);
    double Q = sqrt(1 + 2 * e2*e2*P);
    double ro = -(P*e2*r) / (1 + Q) + sqrt((a*a / 2)*(1 + 1 / Q) - (P*(1 - e2)*z2) / (Q*(1 + Q)) - P*r2 / 2);
    double tmp = (r - e2*ro)*(r - e2*ro);
    double U = sqrt(tmp + z2);
    double V = sqrt(tmp + (1 - e2)*z2);
    double zo = (b2*z) / (a*V);

    double height = U*(1 - b2 / (a*V));

    double lat = atan((z + ep*ep*zo) / r);

    double temp = atan(y / x);
    double long_;
    if (x >= 0)
      long_ = temp;
    else if ((x < 0) && (y >= 0))
      long_ = pi + temp;
    else
      long_ = temp - pi;
    llh(0) = (long_)*(180 / pi);
    llh(1) = (lat)*(180 / pi);
    llh(2) = height;
    return llh;

    /**************for test purpose*************************
    Eigen::MatrixXd ecef;
    ecef.resize(3, 1);
    Eigen::MatrixXd llh;
    llh.resize(3, 1);
    ecef(0) = -2418080.9387265667;
    ecef(1) = 5386190.3905763263;
    ecef(2) = 2405041.9305451373;
    llh = ecef2llh(ecef);
    cout << "llh ->: " << llh << "\n";
    */
  }

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: ecef to enu
  input: original llh, and current ecef (Matrix3d)
  output: enu (Matrix3d)
  */
  Eigen::MatrixXd ecef2enu(Eigen::MatrixXd originllh, Eigen::MatrixXd ecef) // transform the ecef to enu 
  {
    double pi = 3.1415926; // pi 
    double DEG2RAD = pi / 180.0;
    double RAD2DEG = 180.0 / pi;
    
    Eigen::MatrixXd enu; // the enu for output
    enu.resize(3, 1); // resize to 3X1
    Eigen::MatrixXd oxyz; // the original position 
    oxyz.resize(3, 1); // resize to 3X1

    double x, y, z; // save the x y z in ecef
    x = ecef(0);
    y = ecef(1);
    z = ecef(2);

    double ox, oy, oz; // save original reference position in ecef
    oxyz = llh2ecef(originllh);
    ox = oxyz(0); // obtain x in ecef 
    oy = oxyz(1); // obtain y in ecef
    oz = oxyz(2); // obtain z in ecef

    double dx, dy, dz;
    dx = x - ox;
    dy = y - oy;
    dz = z - oz;

    double lonDeg, latDeg, _; // save the origin lon alt in llh
    lonDeg = originllh(0);
    latDeg = originllh(1);
    double lon = lonDeg * DEG2RAD;
    double lat = latDeg * DEG2RAD;

    //save ENU
    enu(0) = -sin(lon) * dx + cos(lon) * dy;
    enu(1) = -sin(lat) * cos(lon) * dx - sin(lat) * sin(lon) * dy + cos(lat) * dz;
    enu(2) = cos(lat) * cos(lon) * dx + cos(lat) * sin(lon) * dy + sin(lat) * dz;
    return enu;

    /**************for test purpose*****suqare distance is about 37.4 meters********************
    Eigen::MatrixXd llh;  //original
    llh.resize(3, 1);
    llh(0) = 114.1775072541416;
    llh(1) = 22.29817969722738;
    llh(2) = 58;
    Eigen::MatrixXd ecef;
    ecef.resize(3, 1);
    ecef(0) = -2418080.9387265667;
    ecef(1) = 5386190.3905763263;
    ecef(2) = 2405041.9305451373;
    Eigen::MatrixXd enu;
    enu.resize(3, 1);
    enu = ecef2enu(llh, ecef);
    cout << "enu ->: " << enu << "\n";
    */
  }

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk) 
  function: identify if one point (given in ENU) is inside a building
  input: map mapInput, Eigen::MatrixXd ENU
  output: yes or no (bool)
  */
  bool identifyPointInsideBuildingENU(map mapInput, Eigen::MatrixXd ENU) // identify if one point is inside the building (ENU)
  {
    // 
    bool result=0; // 1: inside a builidng
    Point_  ENUPoint = { ENU (0),ENU(1)}; // prepare the pont needed to be identified

    //index all the buildings
    for (int i = 0; i < mapInput.buildingS_M.size(); i++) // index all the buildings in the map struct 
    {
        building building_; // processed: a buidling information
        Point_ pointTemp; // temp 
        vector<Point_> buildingPolygon; // save all the convex hull node 
        for (int j = 0; j < mapInput.buildingS_M[i].buildingHull.size(); j++) // index the every point in building hull (ENU) in one buildings
        {
          //cout << " ENU--------mapInput.buildingS_M[i].buildingHull[j].x> " << mapInput.buildingS_M[i].buildingHull[j].x << "       ENU--------mapInput.buildingS_M[i].buildingHull[j].x> " << mapInput.buildingS_M[i].buildingHull[j].y<<"\n";
          pointTemp.x = mapInput.buildingS_M[i].buildingHull[j].x;
          pointTemp.y = mapInput.buildingS_M[i].buildingHull[j].y;
          buildingPolygon.push_back(pointTemp);
        }
        if (isInside(buildingPolygon, buildingPolygon.size(), ENUPoint))
        {
          //isInside(buildingPolygon, buildingPolygon.size(), ENUPoint) ? cout << "Yes in building ->:  \n" : cout << "Not in building \n";
          return isInside(buildingPolygon, buildingPolygon.size(), ENUPoint);
        }
          
        //isInside(buildingPolygon, buildingPolygon.size(), ENUPoint) ? cout << "Yes in building ->:  \n" : cout << "Not in building \n";
        //cout << "mapInput.buildingS_M[i].buildingHull.size()--------> " << mapInput.buildingS_M[i].buildingHull.size() << "\n\n";
    }



    /**************for test purpose*************************
    vector<Point_> polygon1 = { { 0, 0 },{ 10, 0 },{ 10, 10 },{ 0, 10 } };
    //int n = sizeof(polygon1) / sizeof(polygon1[0]);
    int n = polygon1.size();
    Point_ p = { 20, 20 };
    isInside(polygon1, n, p) ? cout << "Yes \n" : cout << "No \n";

    p = { 5, 5 };
    isInside(polygon1, n, p) ? cout << "Yes \n" : cout << "No \n";

    vector<Point_> polygon2 = { { 0, 0 },{ 5, 5 },{ 5, 0 } };
    p = { 3, 3 };
    //n = sizeof(polygon2) / sizeof(polygon2[0]);
    n = polygon2.size();
    isInside(polygon2, n, p) ? cout << "Yes \n" : cout << "No \n";

    p = { 5, 1 };
    isInside(polygon2, n, p) ? cout << "Yes \n" : cout << "No \n";

    p = { 8, 1 };
    isInside(polygon2, n, p) ? cout << "Yes \n" : cout << "No \n";

    vector<Point_> polygon3 = { { 0, 0 },{ 10, 0 },{ 10, 10 },{ 0, 10 } };
    p = { -1,10 };
    //n = sizeof(polygon3) / sizeof(polygon3[0]);
    n = polygon3.size();
    isInside(polygon3, n, p) ? cout << "Yes \n" : cout << "No \n";
    
    */
    return result;
  }

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: identify if one point (given in LLH) is inside a building
  input: map mapInput, Eigen::MatrixXd LLH
  output: yes or no (bool)

  Notation:
    1: change LLH into ENU
    2. use identifyPointInsideBuildingENU subsequently
  */
  bool identifyPointInsideBuildingLLH(map mapInput, Eigen::MatrixXd LLH) // identify if one point is inside the building (LLH)
  {
    // 
    bool result = 0; // 1: inside a builidng
    // obtain ENU
    //****obtain ecef
    Eigen::MatrixXd ecef;
    ecef.resize(3, 1);
    ecef = llh2ecef(LLH); // position in ecef

    //****obtain ENU
    Eigen::MatrixXd llhO;  //original
    llhO.resize(3, 1);
    llhO(0) = mapInput.llhOri(0); //original lon for ENU reference 
    llhO(1) = mapInput.llhOri(1); //original lat for ENU reference 
    llhO(2) = mapInput.llhOri(2); //original alt for ENU reference 
    Eigen::MatrixXd enu;
    enu.resize(3, 1);
    enu = ecef2enu(llhO, ecef);

    Point_  ENUPoint = { enu(0),enu(1) }; // prepare the pont needed to be identified
    result = identifyPointInsideBuildingENU(mapInput, enu);
    return result;
  }

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: get the mask elevation angle at a position with certain azimuth
  input: map mapInput, Eigen::MatrixXd LLH, double azimuth
  output: Eigen::MatrixXd (azimuth, maximum elevation angle , nearest intersect building number)

  */
  Eigen::MatrixXd getMaskElevationFromLLH(map mapInput, Eigen::MatrixXd LLH, double azimuth) // get the maximum elevation angle at a certain position (LLH) with a certain azimuth
  {
    Eigen::MatrixXd result; // result(0)->azimuth  result(1)->elevation 
    result.resize(3,1);
    result(2) = 10000;
    double Maxelevation = 0;
    double minDistance = 10000;

    // obtain ENU
    //****obtain ecef
    Eigen::MatrixXd ecef;
    ecef.resize(3, 1);
    ecef = llh2ecef(LLH); // position in ecef

     //****obtain ENU of the given position
    Eigen::MatrixXd llhO;  //original
    llhO.resize(3, 1);
    llhO(0) = mapInput.llhOri(0); //original lon for ENU reference 
    llhO(1) = mapInput.llhOri(1); //original lat for ENU reference 
    llhO(2) = mapInput.llhOri(2); //original alt for ENU reference 
    Eigen::MatrixXd enu;
    enu.resize(3, 1);
    enu = ecef2enu(llhO, ecef);

    result = getMaskElevationFromENU(mapInput, enu, azimuth);
    return result;
  }

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: get the mask elevation angle at a position with certain azimuth
  input: map mapInput, Eigen::MatrixXd ENU, double azimuth
  output: Eigen::MatrixXd (azimuth, maximum elevation angle  , nearest intersect building number)

  */
  Eigen::MatrixXd getMaskElevationFromENU(map mapInput, Eigen::MatrixXd ENU, double azimuth) // get the maximum elevation angle at a certain position (LLH) with a certain azimuth
  {
    Eigen::MatrixXd result; // result(0)->azimuth  result(1)->elevation 
    result.resize(3, 1);
    double Maxelevation = 0; // maximum elevation of blockage
    double minDistance = 10000;

    // obtain ENU
    Eigen::MatrixXd enu;
    enu.resize(3, 1);
    enu = ENU;

    Eigen::MatrixXd enuEnd; // the segment end
    enuEnd.resize(3, 1);
    enuEnd(0) = enu(0) + mapInput.maxSearDis * sin(azimuth * (3.1415926 / 180.0));
    enuEnd(1) = enu(1) + mapInput.maxSearDis * cos(azimuth * (3.1415926 / 180.0));
    Point_ p1;
    Point_ q1;
    Point_ p2;
    Point_ q2;
    p1.x = enu(0);
    p1.y = enu(1);
    q1.x = enuEnd(0);
    q1.y = enuEnd(1);

    
    ////index all the buildings in the maps
    for (int i = 0; i < mapInput.buildingS_M.size(); i++) // index all the buildings in the map struct 
    {
      building building_; // processed: a buidling information
      Point_ p2; // temp 
      vector<Point_> buildingPolygon; // save all the convex hull node 
                      //#pragma omp parallel for
      for (int j = 0; j < mapInput.buildingS_M[i].buildingENUV.size() - 1; j++) // index the every point in building hull (ENU) in one buildings
      {
        p2.x = mapInput.buildingS_M[i].buildingENUV[j].E;
        p2.y = mapInput.buildingS_M[i].buildingENUV[j].N;

        q2.x = mapInput.buildingS_M[i].buildingENUV[j + 1].E;
        q2.y = mapInput.buildingS_M[i].buildingENUV[j + 1].N;

        if (doIntersect(p1, q1, p2, q2)) // identify if intersect between line segment p1q1 and line segment p2q2
        {
          // intersection is available

          result.resize(3, 1);
          Point_ intersection;
          intersection = lineLineIntersection(p1, q1, p2, q2); // obtain the intersection in ENU
          double height = mapInput.buildingS_M[i].buildingENUV[j].U;
          double disP12Insec = disPP(p1, intersection); // distance from p1 to intersection
          if (disP12Insec < minDistance)  result(2) = i; // the building with shortest distance to point with intersection
          double elevation_ = 180.0 / 3.1415926 * atan(height / disP12Insec); // calculate the elevation 
          //cout << "intersection available---------------------------------------------------: "<< disP12Insec<<"       elevationMax: "<< elevation_ <<"\n\n";
          if (elevation_ > Maxelevation)  Maxelevation = elevation_;
        }

      }
    }
    result(0) = azimuth;
    result(1) = Maxelevation;
    //cout << "azimuth: " << azimuth << "             Maxelevation---------------------------------: " << Maxelevation << "\n\n";
    return result;
  }

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: create skymask 
  input: map mapInput
  output: skymasks

  */
  void createSkymasks() // generate all the skymask
  {
    //generater all the skymasks
    std::cout << "[Function]->createSkymasks begin generate skymask .........." << std::endl;
    //_sleep(10000); // pauses for 10 seconds
    double deltaE = map_.ENUMax(0) - map_.ENUMin(0);
    std::cout<<"map_.ENUMax(0)->"<<map_.ENUMax(0)<<"     map_.ENUMin(0)"<<map_.ENUMin(0)<<std::endl;
    double deltaN = map_.ENUMax(1) - map_.ENUMin(1);
    double skymaskNum = deltaN*deltaE / (pow(map_.disReso, 2));
    double finishNum = 0;
    cout << "delta E ->:  " << deltaE << "             delta N->" << deltaN<<"\n\n\n";
    wait(1);
    for (double N = map_.ENUMin(1); N <= map_.ENUMax(1); N = N + map_.disReso) // search in the by line (N direction)
    {
      for (double E = map_.ENUMin(0); E <=map_.ENUMax(0); E = E + map_.disReso) // search in the by line (E direction)
      {
    //for (double N = map_.ENUMin(1); N <= map_.ENUMin(1) + 3; N = N + map_.disReso) // search in the by line (N direction)
    //{
    //  for (double E = map_.ENUMin(0); E <= map_.ENUMin(0) + 3; E = E + map_.disReso) // search in the by line (E direction)
    //  {
        bool insideBuildings = 0; // inside buildings ?
        // generate the skymask of one position in 360 degree
        Eigen::MatrixXd ENU;
        ENU.resize(1, 3);
        ENU(0) = E;
        ENU(1) = N;
        ENU(2) = 0;
        skyMask skyMask_;

        const clock_t begin_time = clock();

        // identify if this point is inside the building first
        if (identifyPointInsideBuildingENU(map_, ENU)) // if a point is inside a building, set elevation (result(1)) to -1.
        {
          insideBuildings = 1; // this justification rule sometimes does not wok,
        }

        if (insideBuildings != 1) // not inside the buildings
        {
          cout << "outside the building      ";
          for (double azi = 0; azi <= 360; azi = azi + map_.aziReso)
          {
            Eigen::MatrixXd aziEleResult;
            aziEleResult.resize(3, 1);
            elevationMask elevation_;
            aziEleResult = getMaskElevationFromENU(map_, ENU, azi);
            elevation_.azimuth = aziEleResult(0);
            elevation_.elevation = aziEleResult(1);

            skyMask_.poseENU.E = ENU(0);
            skyMask_.poseENU.N = ENU(1);
            skyMask_.poseENU.U = ENU(2);
            skyMask_.insideBuilding = 0; // outside the building 
            skyMask_.aziElemask.push_back(elevation_);
          }
        }
        if (insideBuildings == 1) // inside the buildings
        {
          cout << "inside the building      ";
          for (double azi = 0; azi <= 360; azi = azi + map_.aziReso)
          {
            Eigen::MatrixXd aziEleResult;
            aziEleResult.resize(2, 1);
            elevationMask elevation_;
            aziEleResult(0) = azi;
            aziEleResult(1) = -1;
            elevation_.azimuth = aziEleResult(0);
            elevation_.elevation = aziEleResult(1);

            skyMask_.poseENU.E = ENU(0);
            skyMask_.poseENU.N = ENU(1);
            skyMask_.poseENU.U = ENU(2);
            skyMask_.insideBuilding = 1; // inside the building
            skyMask_.aziElemask.push_back(elevation_);
          }
        }
        
        finishNum++;
        skyMaskS.push_back(skyMask_);
        cout <<"total skymasks-> "<< skymaskNum << "finish sky mask->  " << skyMaskS.size() << "      rest skymask-> "<< skymaskNum - finishNum;
        std::cout << "       this sky mask used  time -> " << float(clock() - begin_time) / CLOCKS_PER_SEC << "\n\n";
      }
    }
    cout << "finishing generate skymask...\n";
      

    
  }

  //void saveSkymasks2SCV(map mapInput) // save all the skymask into a csv file
  //{
  //  cout << "save skymask to csv file" << "\n";
  //  std::ofstream out("skymask.txt");
  //  std::ostringstream strs;
  //  // index the 
  //  for (int grid = 0; grid <skyMaskS.size(); grid = grid + 1) // index grid
  //  {
  //    strs << "<skymask>";
  //    strs << "\n";
  //    strs << skyMaskS[grid].poseENU.E;
  //    strs << ",";
  //    strs << skyMaskS[grid].poseENU.N;
  //    strs << "\n";
  //    for (double sk = 0; sk < skyMaskS[grid].aziElemask.size(); sk = sk + 1) // index skymask
  //    {
  //      //string content;
  //      strs << skyMaskS[grid].aziElemask[sk].azimuth;// save azimuth
  //      strs << ",";
  //      strs<< skyMaskS[grid].aziElemask[sk].elevation;  // save elevation
  //      strs << ",";
  //      strs << skyMaskS[grid].insideBuilding;  // inside building
  //      strs << "  ";
  //    }
  //    strs << "\n";
  //    strs << "<skymask>";
  //    strs << "\n";
  //    strs << "\n";
  //  }
  //  string head; // save the ENU
  //  head = strs.str();
  //  out << head;
  //  out.close(); // finish save file
  //  
  //  
  //
  //  ////save data to txt (from double to string first)
  //  //std::ofstream out("output.txt");
  //  //std::ostringstream strs;
  //  //double a = value * 100000;
  //  //strs << a;
  //  //std::string str = strs.str();
  //  //out << str;
  //  //out.close();
  //
  //}

  /*
  author: WEN Weisong (17902061r@connect.polyu.hk)
  function: save skymask to CSV file
  input: map mapInput
  output: skymasks

  */
  void saveSkymasks2SCV(map mapInput,string saved_skymask_dir) // save all the skymask into a csv file
  {
    std::cout << "[Function]->saveSkymasks2SCV save skymask to csv file.........." << std::endl;
    /*std::ofstream out("skymask.txt");
    std::ostringstream strs;*/
    // index the 
    for (int grid = 0; grid <skyMaskS.size(); grid = grid + 1) // index grid £¨one grid one skymask file£©  
    {
      string filename;  
      std::ostringstream filenameStrs; //
      // filenameStrs << "/home/wenws/amsipolyu/src/rtklibros/app/generate_skymask/skymask/";
      filenameStrs << saved_skymask_dir;
      filenameStrs << int(skyMaskS[grid].poseENU.E);
      filenameStrs << " ";
      filenameStrs << int(skyMaskS[grid].poseENU.N);
      filenameStrs << ".txt";
      filename = filenameStrs.str();
      std::ofstream out(filename);
      std::ostringstream strs;
      std::cout << std::setprecision(17);
      double lon = mapInput.altMax;
      strs << map_.llhOri(0);
      strs << ",";
      strs << map_.llhOri(1);
      strs << ",";
      strs << map_.llhOri(2);
      strs << "\n";
      strs << "<skymask>";
      strs << "\n";
      strs << skyMaskS[grid].poseENU.E; // E in ENU
      strs << ",";
      strs << skyMaskS[grid].poseENU.N; // N in ENU
      strs << ",";
      strs << skyMaskS[grid].insideBuilding;  // inside building
      strs << ",";
      strs << 1;  // azimuth resolution
      strs << "\n";
      for (double sk = 0; sk < skyMaskS[grid].aziElemask.size(); sk = sk + 1) // index skymask
      {
        //string content;
        strs << skyMaskS[grid].aziElemask[sk].elevation;  // save elevation
        strs << "  ";
      }
      strs << "\n";
      strs << "<skymask>";
      strs << "\n";
      strs << "\n";

      if (!skyMaskS[grid].insideBuilding)
      {
        string head; // save the ENU
        head = strs.str();
        out << head;
        out.close(); // finish save file
      }

      
    }
    std::cout << "saved all the skymasks to certain directory.........." << std::endl;
  }

private:
  int reserve1;

  bool mannually_set_ref_llh_;
  double ref_lat_fromlaunch;
  double ref_lon_fromlaunch;
  double ref_alt_fromlaunch;
  double ref_ENU_LiDAR_Angle_fromlaunch;

private:
  ros::Publisher pub_debug_marker_; // marker publisher
  ros::Publisher marker_pub;
  visualization_msgs::MarkerArray markers; // markers for building models

  

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "buildingModelLoader");
  std::cout<<"buildingModelLoader......"<<std::endl;

  // get parameters
  std::string directory,saved_skymask_dir;
  int skymask_grid_resolution = 2;

  bool mannually_set_ref_llh = false;
  double ref_lat = 0.0;
  double ref_lon = 0.0;
  double ref_alt = 0.0;
  double ENU_LiDAR_Angle = 0.0;

  ros::NodeHandle nh_private("~");

  nh_private.param<std::string>("directory", directory, "/home/wws/Downloads/midundao20190606/MongKok.kml"); 
  nh_private.param<int>("skymask_grid_resolution", skymask_grid_resolution, 2); 
  nh_private.param<std::string>("saved_skymask_dir", saved_skymask_dir, "/home/wws/Downloads/midundao20190606/skymask/"); 
  
  nh_private.param<bool>("mannually_set_ref_llh", mannually_set_ref_llh, false); 
  nh_private.param<double>("ref_lat", ref_lat, 0.0); 
  nh_private.param<double>("ref_lon", ref_lon, 0.0); 
  nh_private.param<double>("ref_alt", ref_alt, 0.0); 
  nh_private.param<double>("ENU_LiDAR_Angle", ENU_LiDAR_Angle, 0.0); 



  // printf("obss.n = %d\n", obss.n);
  buildingModelLoader buildingModelLoader_(directory,saved_skymask_dir,mannually_set_ref_llh, ref_lat,ref_lon,ref_alt,ENU_LiDAR_Angle);
  ros::spinOnce();
  while (ros::ok()) {

  }
  return 0;
}