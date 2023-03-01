#include <interp_cable/utilities.hpp>

vector<double> linspace(double start, double end, int n) {
    vector<double> result;
    double step = (end - start) / (n - 1);
    for (int i = 0; i < n; i++) {
        result.push_back(start + step * i);
    }
    return result;
}

int find_closest_point(vector<vector<double>> &pointset, vector<double> &point) {
    int index = 0;
    double min_dist = 1e10;
    for (int i = 0; i < pointset.size(); i++) {
        double dist = 0;
        for (int j = 0; j < pointset[i].size(); j++) {
            dist += pow(pointset[i][j] - point[j], 2);
        }
        if (dist < min_dist) {
            min_dist = dist;
            index = i;
        }
    }
    return index;
}