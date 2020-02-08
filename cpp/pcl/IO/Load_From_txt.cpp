#include "Load_From_txt.h"

using namespace std;

void Load_From_TXT() {

    pcl::PointCloud<pcl::PointXYZ> cloudy;

    string line;
    ifstream xfile("x.txt");
    ifstream yfile("y.txt");
    ifstream zfile("z.txt");

    vector<float> xv;
    vector<float> yv;
    vector<float> zv;

    if (xfile.is_open()) {
        while (getline(xfile, line)) {
            xv.push_back(std::atof(line.c_str()));
        }
        xfile.close();
    } else cout << "xNO";

    if (yfile.is_open()) {
        while (getline(yfile, line)) {
            yv.push_back(std::atof(line.c_str()));
        }
        yfile.close();
    } else cout << "yNO";

    if (zfile.is_open()) {
        while (getline(zfile, line)) {
            zv.push_back(std::atof(line.c_str()));
        }
        zfile.close();
    } else cout << "ZNO";


// Fill in the cloud data
    cloudy.width = 1300;
    cloudy.height = 1;
    cloudy.is_dense = false;
    cloudy.points.resize(cloudy.width * cloudy.height);

    for (size_t i = 0; i < cloudy.points.size(); ++i) {
        cloudy.points[i].x = xv[i];
        cloudy.points[i].y = yv[i];
        cloudy.points[i].z = -zv[i];
    }

    pcl::io::savePCDFileASCII("big.pcd", cloudy);
    std::cerr << "Saved " << cloudy.points.size() << " data points to test_pcd.pcd." << std::endl;

    for (size_t i = 0; i < cloudy.points.size(); ++i)
        std::cerr << "    " << cloudy.points[i].x << " " << cloudy.points[i].y << " " << cloudy.points[i].z
                  << std::endl;
}