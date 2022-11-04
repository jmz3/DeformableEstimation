#include <sorting/paraspline.hpp>

Cable::paraspline::paraspline() : m_type(cubic), m_left(second_order), m_right(second_order),
                                  m_left_value(0.0), m_right_value(0.0){

                                                     };

Cable::paraspline::paraspline(const std::vector<double> &X, const std::vector<double> &Y,
                              spline_type type = cubic,
                              bool make_monotonic = false,
                              bound_type left = second_order,
                              double left_value = 0.0,
                              bound_type right = second_order,
                              double right_value = 0.0) : m_type(type),
                                                          m_left(left),
                                                          m_right(right),
                                                          m_left_value(left_value),
                                                          m_right_value(right_value)
{
    this->set_points(X, Y, m_type);
};

void Cable::paraspline::set_boundary(bound_type left, double left_value,
                                     bound_type right, double right_value)
{
    ROS_ASSERT(m_x.size() == 0);
    m_left = left;
    m_right = right;
    m_left_value = left_value;
    m_right_value = right_value;
}

void Cable::paraspline::set_points(const std::vector<double> &x,
                                   const std::vector<double> &y,
                                   spline_type type)
{
    ROS_ASSERT(x.size() == y.size());
    ROS_ASSERT(x.size() >= 3);

    // If points are less than 3, there will be multiple solutions?
    m_type = type;
    m_x = x;
    m_y = y;
    int n = (int)x.size();

    // Ensure the input independent variable to be monotonic
    for (int i = 0; i < n - 1; i++)
    {
        ROS_ASSERT(m_x[i] < m_x[i + 1]);
    }

    switch (type)
    {
    case linear:
        m_d.resize(n);
        m_c.resize(n);
        m_b.resize(n);

        for (int i = 0; i < n - 1; i++)
        {
            m_d[i] = 0.0;
            m_c[i] = 0.0;
            m_b[i] = (m_y[i + 1] - m_y[i]) / (m_x[i + 1] - m_x[i]);
        }
        // ignore boundary conditions, set slope equal to the last segment
        m_b[n - 1] = m_b[n - 2];
        m_c[n - 1] = 0.0;
        m_d[n - 1] = 0.0;
        break;

    case cubic:
        // classical cubic splines which are C^2 (twice cont differentiable)
        // this requires solving an equation system

        // setting up the matrix and right hand side of the equation system
        // for the parameters b[]
        // conditional operator: expr ? true_return : false_return
        int n_upper = (m_left == paraspline::not_a_knot) ? 2 : 1;
        int n_lower = (m_right == paraspline::not_a_knot) ? 2 : 1;

        band_matrix A(n, n_upper, n_lower);
        std::vector<double> rhs(n);
        for (int i = 1; i < n - 1; i++)
        {
            A(i, i - 1) = 1.0 / 3.0 * (x[i] - x[i - 1]);
            A(i, i) = 2.0 / 3.0 * (x[i + 1] - x[i - 1]);
            A(i, i + 1) = 1.0 / 3.0 * (x[i + 1] - x[i]);

            rhs[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
        }

        if (m_left == paraspline::second_order)
        { // 2 * c[0] = f"(x0)
            A(0, 0) = 2.0;
            A(0, 1) = 0.0;
            rhs[0] = m_left_value;
        }
        else if (m_left == paraspline::first_order)
        {
            // b[n-1] = f', needs to be re-expressed in terms of c:
            // (c[n-2]+2c[n-1])(x[n-1]-x[n-2])
            // = 3 (f' - (y[n-1]-y[n-2])/(x[n-1]-x[n-2]))
            A(n - 1, n - 1) = 2.0 * (x[n - 1] - x[n - 2]);
            A(n - 1, n - 2) = 1.0 * (x[n - 1] - x[n - 2]);
            rhs[n - 1] = 3.0 * (m_right_value - (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]));
        }
        else if (m_left == paraspline::not_a_knot)
        {
            // f'''(x[1]) exists, i.e. d[0]=d[1], or re-expressed in c:
            // -h1*c[0] + (h0+h1)*c[1] - h0*c[2] = 0
            A(0, 0) = -(x[2] - x[1]);
            A(0, 1) = x[2] - x[0];
            A(0, 2) = -(x[1] - x[0]);
            rhs[0] = 0.0;
        }
        else
        {
            ROS_ASSERT(false);
        }
        if (m_right == paraspline::second_order)
        {
            A(n - 1, n - 2) = 2.0;
            A(n - 1, n - 2) = 0.0;
            rhs[n - 1] = m_right_value;
        }
        else if (m_right == paraspline::first_order)
        {
            // b[n-1] = f', needs to be re-expressed in terms of c:
            // (c[n-2]+2c[n-1])(x[n-1]-x[n-2])
            // = 3 (f' - (y[n-1]-y[n-2])/(x[n-1]-x[n-2]))
            A(n - 1, n - 1) = 2.0 * (x[n - 1] - x[n - 2]);
            A(n - 1, n - 2) = 1.0 * (x[n - 1] - x[n - 2]);
            rhs[n - 1] = 3.0 * (m_right_value - (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]));
        }
        else if (m_right == paraspline::not_a_knot)
        {
            // f'''(x[n-2]) exists, i.e. d[n-3]=d[n-2], or re-expressed in c:
            // -h_{n-2}*c[n-3] + (h_{n-3}+h_{n-2})*c[n-2] - h_{n-3}*c[n-1] = 0
            A(n - 1, n - 3) = -(x[n - 1] - x[n - 2]);
            A(n - 1, n - 2) = x[n - 1] - x[n - 3];
            A(n - 1, n - 1) = -(x[n - 2] - x[n - 3]);
            rhs[0] = 0.0;
        }
        else
        {
            ROS_ASSERT(false);
        }
        break;
    }
    m_c0 = (m_left == first_order) ? 0.0 : m_c[0];
}

size_t Cable::paraspline::find_closest(double x) const
{
    std::vector<double>::const_iterator it;
    it = std::upper_bound(m_x.begin(), m_x.end(), x);
    size_t idx = std::max(int(it - m_x.begin()) - 1, 0);
    return idx;
}

double Cable::paraspline::operator()(double x) const
{
    // polynomial evaluation using Horner's scheme
    // TODO: consider more numerically accurate algorithms, e.g.:
    //   - Clenshaw
    //   - Even-Odd method by A.C.R. Newbery
    //   - Compensated Horner Scheme
    size_t n = m_x.size();
    size_t idx = find_closest(x);
    double h = x - m_x[idx];
    double interpol;
    if (x < m_x[0])
    {
        // extrapolation to the left
        interpol = (m_c0 * h + m_b[0]) * h + m_y[0];
    }
    else if (x > m_x[n - 1])
    {
        // extrapolation to the right
        interpol = (m_c[n - 1] * h + m_b[n - 1]) * h + m_y[n - 1];
    }
    else
    {
        // interpolation
        interpol = ((m_d[idx] * h + m_c[idx]) * h + m_b[idx]) * h + m_y[idx];
    }
    return interpol;
}

double Cable::paraspline::deriv(int order, double x) const
{
    ROS_ASSERT(order > 0);
    size_t n = m_x.size();
    size_t idx = find_closest(x);

    double h = x - m_x[idx];
    double interpol;
    if (x < m_x[0])
    {
        switch (order)
        {
        case 1:
            interpol = 2.0 * m_c0 * h + m_b[0];
            break;

        case 2:
            interpol = 2.0 * m_c0;
            break;

        default:
            interpol = 0.0;
            break;
        }
    }
    else if (x > m_x[n - 1])
    {
        switch (order)
        {
        case 1:
            interpol = 2.0 * m_c[n - 1] * h + m_b[n - 1];
            break;

        case 2:
            interpol = 2.0 * m_c[n - 1];
            break;

        default:
            interpol = 0.0;
            break;
        }
    }
}