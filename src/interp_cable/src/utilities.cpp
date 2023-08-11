#include <interp_cable/utilities.hpp>

vector<double> Linspace(double start, double end, int num) {
    vector<double> result;
    double step = (end - start) / (num - 1);
    for (int i = 0; i < num; i++) {
        result.push_back(start + step * i);
    }
    return result;
}

int FindClosestPoint(vector<vector<double>> &pointset, vector<double> &point) {
    int kIndex = 0;
    double kMin_dist = 1e10;
    for (int i = 0; i < pointset.size(); i++) {
        double kDist = 0;
        for (int j = 0; j < pointset[i].size(); j++) {
            kDist += pow(pointset[i][j] - point[j], 2);
        }
        if (kDist < kMin_dist) {
            kMin_dist = kDist;
            kIndex = i;
        }
    }
    return kIndex;
}

void PrintVector(vector<double> &v)
{
    for (int i = 0; i < v.size(); i++)
    {
        cout << v[i] << ", ";
    }
    cout << endl;
}