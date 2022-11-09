#include <sorting/utilities.hpp>

vector<double> linspace(double start, double end, int n) {
    vector<double> result;
    double step = (end - start) / (n - 1);
    for (int i = 0; i < n; i++) {
        result.push_back(start + step * i);
    }
    return result;
}