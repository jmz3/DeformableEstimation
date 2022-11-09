#include <cstdio>
#include <cassert>
#include <vector>
#include <cmath>
#include <ros/ros.h>

// f(x) = a_i + b_i*(x-x_i) + c_i*(x-x_i)^2 + d_i*(x-x_i)^3
// where a_i = y_i, or else it won't go through grid points

namespace Cable
{
    class paraspline
    {
    public:
        // interpolation types
        enum spline_type
        {
            linear = 10, // Linear Interpolation
            cubic = 30,  // Cubic Spline C2 continuous
        };

        // Boundary Condition type
        enum bound_type
        {
            first_order = 1,  // Given 1st order boundary as initial condition
            second_order = 2, // Given 2nd order boundary as initial condition
            not_a_knot = 3    //
        };

    protected:
        std::vector<double> m_x, m_y; // input x, y data
        // interpolation parameters

        std::vector<double> m_b, m_c, m_d; // spline coefficients
        double m_c0;

        spline_type m_type;
        bound_type m_left, m_right;
        double m_left_value, m_right_value;
        void set_coeffs_from_b();
        size_t find_closest(double x) const;

    public:
        paraspline();
        paraspline(const std::vector<double> &X, const std::vector<double> &Y,
                   spline_type type,
                   bool make_monotonic,
                   bound_type left,
                   double left_value,
                   bound_type right,
                   double right_value);

        // modify boundary conditions: if called it must be before set_points()
        void set_boundary(bound_type left, double left_value,
                          bound_type right, double right_value);

        // set all data points (cubic_spline=false means linear interpolation)
        void set_points(const std::vector<double> &x,
                        const std::vector<double> &y,
                        spline_type type);

        // adjust coefficients so that the spline becomes piecewise monotonic
        // where possible
        //   this is done by adjusting slopes at grid points by a non-negative
        //   factor and this will break C^2
        //   this can also break boundary conditions if adjustments need to
        //   be made at the boundary points
        // returns false if no adjustments have been made, true otherwise

        // evaluates the spline at point x
        double operator()(double x) const;
        double deriv(int order, double x) const;

        // solves for all x so that: spline(x) = y
        std::vector<double> solve(double y, bool ignore_extrapolation = true) const;

        // returns the input data points
        std::vector<double> get_x() const { return m_x; }
        std::vector<double> get_y() const { return m_y; }
        double get_x_min() const
        {
            ROS_ASSERT(!m_x.empty());
            return m_x.front();
        }
        double get_x_max() const
        {
            ROS_ASSERT(!m_x.empty());
            return m_x.back();
        }
    };

    class band_matrix
    {
    private:
        std::vector<std::vector<double>> m_upper;
        std::vector<std::vector<double>> m_lower;

    public:
        band_matrix(){};
        band_matrix(int dim, int n_u, int n_l);
        ~band_matrix(){};
        void resize(int dim, int n_u, int n_l);
        int dim() const;
        int num_upper() const
        {
            return (int)m_upper.size() - 1;
        }
        int num_lower() const
        {
            return (int)m_lower.size() - 1;
        }
        // access operator
        double &operator()(int i, int j);      // write
        double operator()(int i, int j) const; // read
        double &saved_diag(int i);
        double saved_diag(int i) const;
        void lu_decompose();
        std::vector<double> r_solve(const std::vector<double> &b) const;
        std::vector<double> l_solve(const std::vector<double> &b) const;
        std::vector<double> lu_solve(const std::vector<double> &b,
                                     bool is_lu_decomposed = false);
    };
    double get_eps();

    std::vector<double> solve_linear(double a, double b);
    std::vector<double> solve_quadratic(double a, double b, double c,
                                    int newton_iter);
    std::vector<double> solve_cubic(double a, double b, double c, double d,
                                    int newton_iter);
};