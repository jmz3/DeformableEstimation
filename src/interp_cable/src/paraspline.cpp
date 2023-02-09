#include <interp_cable/paraspline.hpp>

Cable::paraspline::paraspline() : m_type(cubic), m_left(second_order), m_right(second_order),
                                  m_left_value(0.0), m_right_value(0.0){

                                                     };

Cable::paraspline::paraspline(const std::vector<double> &X,
                              const std::vector<double> &Y,
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
    // ROS_ASSERT(m_x.size() == 0);
    m_left = left;
    m_right = right;
    m_left_value = left_value;
    m_right_value = right_value;
}

void Cable::paraspline::set_points(const std::vector<double> &x,
                                   const std::vector<double> &y,
                                   spline_type type = cubic)
{
    ROS_ASSERT(x.size()==y.size());
    ROS_ASSERT(x.size()>=3);
    // not-a-knot with 3 points has many solutions
    if(m_left==not_a_knot || m_right==not_a_knot)
        ROS_ASSERT(x.size()>=4);
    m_type=type;

    m_x=x;
    m_y=y;
    int n = (int) x.size();
    // check strict monotonicity of input vector x
    for(int i=0; i<n-1; i++) {
        ROS_ASSERT(m_x[i]<m_x[i+1]);
    }


    if(type==linear) {
        // linear interpolation
        m_d.resize(n);
        m_c.resize(n);
        m_b.resize(n);
        for(int i=0; i<n-1; i++) {
            m_d[i]=0.0;
            m_c[i]=0.0;
            m_b[i]=(m_y[i+1]-m_y[i])/(m_x[i+1]-m_x[i]);
        }
        // ignore boundary conditions, set slope equal to the last segment
        m_b[n-1]=m_b[n-2];
        m_c[n-1]=0.0;
        m_d[n-1]=0.0;
    } else if(type==cubic) {
        // classical cubic splines which are C^2 (twice cont differentiable)
        // this requires solving an equation system

        // setting up the matrix and right hand side of the equation system
        // for the parameters b[]
        int n_upper = (m_left  == paraspline::not_a_knot) ? 2 : 1;
        int n_lower = (m_right == paraspline::not_a_knot) ? 2 : 1;
        band_matrix A(n,n_upper,n_lower);
        std::vector<double>  rhs(n);
        for(int i=1; i<n-1; i++) {
            A(i,i-1)=1.0/3.0*(x[i]-x[i-1]);
            A(i,i)=2.0/3.0*(x[i+1]-x[i-1]);
            A(i,i+1)=1.0/3.0*(x[i+1]-x[i]);
            rhs[i]=(y[i+1]-y[i])/(x[i+1]-x[i]) - (y[i]-y[i-1])/(x[i]-x[i-1]);
        }
        // boundary conditions
        if(m_left == paraspline::second_order) {
            // 2*c[0] = f''
            A(0,0)=2.0;
            A(0,1)=0.0;
            rhs[0]=m_left_value;
        } else if(m_left == paraspline::first_order) {
            // b[0] = f', needs to be re-expressed in terms of c:
            // (2c[0]+c[1])(x[1]-x[0]) = 3 ((y[1]-y[0])/(x[1]-x[0]) - f')
            A(0,0)=2.0*(x[1]-x[0]);
            A(0,1)=1.0*(x[1]-x[0]);
            rhs[0]=3.0*((y[1]-y[0])/(x[1]-x[0])-m_left_value);
        } else if(m_left == paraspline::not_a_knot) {
            // f'''(x[1]) exists, i.e. d[0]=d[1], or re-expressed in c:
            // -h1*c[0] + (h0+h1)*c[1] - h0*c[2] = 0
            A(0,0) = -(x[2]-x[1]);
            A(0,1) = x[2]-x[0];
            A(0,2) = -(x[1]-x[0]);
            rhs[0] = 0.0;
        } else {
            ROS_ASSERT(false);
        }
        if(m_right == paraspline::second_order) {
            // 2*c[n-1] = f''
            A(n-1,n-1)=2.0;
            A(n-1,n-2)=0.0;
            rhs[n-1]=m_right_value;
        } else if(m_right == paraspline::first_order) {
            // b[n-1] = f', needs to be re-expressed in terms of c:
            // (c[n-2]+2c[n-1])(x[n-1]-x[n-2])
            // = 3 (f' - (y[n-1]-y[n-2])/(x[n-1]-x[n-2]))
            A(n-1,n-1)=2.0*(x[n-1]-x[n-2]);
            A(n-1,n-2)=1.0*(x[n-1]-x[n-2]);
            rhs[n-1]=3.0*(m_right_value-(y[n-1]-y[n-2])/(x[n-1]-x[n-2]));
        } else if(m_right == paraspline::not_a_knot) {
            // f'''(x[n-2]) exists, i.e. d[n-3]=d[n-2], or re-expressed in c:
            // -h_{n-2}*c[n-3] + (h_{n-3}+h_{n-2})*c[n-2] - h_{n-3}*c[n-1] = 0
            A(n-1,n-3) = -(x[n-1]-x[n-2]);
            A(n-1,n-2) = x[n-1]-x[n-3];
            A(n-1,n-1) = -(x[n-2]-x[n-3]);
            rhs[0] = 0.0;
        } else {
            ROS_ASSERT(false);
        }

        // solve the equation system to obtain the parameters c[]
        m_c=A.lu_solve(rhs);

        // calculate parameters b[] and d[] based on c[]
        m_d.resize(n);
        m_b.resize(n);
        for(int i=0; i<n-1; i++) {
            m_d[i]=1.0/3.0*(m_c[i+1]-m_c[i])/(x[i+1]-x[i]);
            m_b[i]=(y[i+1]-y[i])/(x[i+1]-x[i])
                   - 1.0/3.0*(2.0*m_c[i]+m_c[i+1])*(x[i+1]-x[i]);
        }
        // for the right extrapolation coefficients (zero cubic term)
        // f_{n-1}(x) = y_{n-1} + b*(x-x_{n-1}) + c*(x-x_{n-1})^2
        double h=x[n-1]-x[n-2];
        // m_c[n-1] is determined by the boundary condition
        m_d[n-1]=0.0;
        m_b[n-1]=3.0*m_d[n-2]*h*h+2.0*m_c[n-2]*h+m_b[n-2];   // = f'_{n-2}(x_{n-1})
        if(m_right==first_order)
            m_c[n-1]=0.0;   // force linear extrapolation

    } 
    else {
        ROS_ASSERT(false);
    }

    // for left extrapolation coefficients
    m_c0 = (m_left==first_order) ? 0.0 : m_c[0];
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

        // std::cout << "idx: " << idx << std::endl;
        // std::cout << "h: " << h << std::endl;
        // std::cout << "m_d[idx]: " << m_d[idx] << std::endl;
        // std::cout << "m_c[idx]: " << m_c[idx] << std::endl;
        // std::cout << "m_b[idx]: " << m_b[idx] << std::endl;

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
    else
    {
        switch (order)
        {
        case 1:
            interpol = (3.0 * m_d[idx] * h + 2.0 * m_c[idx]) * h + m_b[idx];
            break;
        case 2:
            interpol = 6.0 * m_d[idx] * h + 2.0 * m_c[idx];
            break;

        case 3:
            interpol = 6.0 * m_d[idx];
            break;
        default:
            interpol = 0.0;
            break;
        }
    }
    return interpol;
}

std::vector<double> Cable::paraspline::solve(double y, bool ignore_extrapolation) const
{
    std::vector<double> x;
    std::vector<double> root;
    const size_t n = m_x.size();

    // left extrapolation
    if (ignore_extrapolation == false)
    {
        root = Cable::solve_cubic(m_y[0] - y, m_b[0], m_c0, 0.0, 1);
        for (size_t j = 0; j < root.size(); j++)
        {
            if (root[j] < 0.0)
            {
                x.push_back(m_x[0] + root[j]);
            }
        }
    }

    // brute force check if piecewise cubic has roots in their resp. segment
    // TODO: make more efficient
    for (size_t i = 0; i < n - 1; i++)
    {
        root = Cable::solve_cubic(m_y[i] - y, m_b[i], m_c[i], m_d[i], 1);
        for (size_t j = 0; j < root.size(); j++)
        {
            double h = (i > 0) ? (m_x[i] - m_x[i - 1]) : 0.0;
            double eps = Cable::get_eps() * 512.0 * std::min(h, 1.0);
            if ((-eps <= root[j]) && (root[j] < m_x[i + 1] - m_x[i]))
            {
                double new_root = m_x[i] + root[j];
                if (x.size() > 0 && x.back() + eps > new_root)
                {
                    x.back() = new_root;
                }
                else
                {
                    x.push_back(new_root);
                }
            }
        }
    }

    // right extrapolation
    if (ignore_extrapolation == false)
    {
        root = Cable::solve_cubic(m_y[n - 1] - y, m_b[n - 1], m_c[n - 1], 0.0, 1);
        for (size_t j = 0; j < root.size(); j++)
        {
            if (0.0 <= root[j])
            {
                x.push_back(m_x[n - 1] + root[j]);
            }
        }
    }

    return x;
}

Cable::band_matrix::band_matrix(int dim, int n_u, int n_l)
{
    resize(dim, n_u, n_l);
}

void Cable::band_matrix::resize(int dim, int n_u, int n_l)
{
    ROS_ASSERT(dim > 0);
    ROS_ASSERT(n_u >= 0);
    ROS_ASSERT(n_l >= 0);
    m_upper.resize(n_u + 1);
    m_lower.resize(n_l + 1);
    for (size_t i = 0; i < m_upper.size(); i++)
    {
        m_upper[i].resize(dim);
    }
    for (size_t i = 0; i < m_lower.size(); i++)
    {
        m_lower[i].resize(dim);
    }
}

int Cable::band_matrix::dim() const
{
    if (m_upper.size() > 0)
    {
        return m_upper[0].size();
    }
    else
    {
        return 0;
    }
}

// defines the new operator (), so that we can access the elements
// by A(i,j), index going from i=0,...,dim()-1
double &Cable::band_matrix::operator()(int i, int j)
{
    int k = i - j;
    ROS_ASSERT((i >= 0) && (i < dim()) && (j >= 0) && (j < dim()));
    ROS_ASSERT((-num_lower() <= k) && (k <= num_upper()));
    // k=0 -> diagonal, k<0 lower left part, k>0 upper right part
    if (k >= 0)
    {
        return m_upper[k][i];
    }
    else
    {
        return m_lower[-k][i];
    }
}

double Cable::band_matrix::operator()(int i, int j) const
{
    int k = j - i; // what band is the entry
    ROS_ASSERT((i >= 0) && (i < dim()) && (j >= 0) && (j < dim()));
    ROS_ASSERT((-num_lower() <= k) && (k <= num_upper()));
    // k=0 -> diagonal, k<0 lower left part, k>0 upper right part
    if (k >= 0)
    {
        return m_upper[k][i];
    }
    else
    {
        return m_lower[-k][i];
    }
}

// second diag (used in LU decomposition). saved in m_lower
double Cable::band_matrix::saved_diag(int i) const
{
    ROS_ASSERT((i >= 0) && (i < dim()));
    return m_lower[0][i];
}

double &Cable::band_matrix::saved_diag(int i)
{
    ROS_ASSERT((i >= 0) && (i < dim()));
    return m_lower[0][i];
}

// LR-Decomposition of a band matrix
void Cable::band_matrix::lu_decompose()
{
    int i_max, j_max;
    int j_min;
    double x;

    // preconditioning
    // normalize column i so that a_ii=1
    for (int i = 0; i < this->dim(); i++)
    {
        ROS_ASSERT(this->operator()(i, i) != 0.0);
        this->saved_diag(i) = 1.0 / this->operator()(i, i);
        j_min = std::max(0, i - this->num_lower());
        j_max = std::min(this->dim() - 1, i + this->num_upper());
        for (int j = j_min; j <= j_max; j++)
        {
            this->operator()(i, j) *= this->saved_diag(i);
        }
        this->operator()(i, i) = 1.0; // prevents rounding errors
    }

    // Gauss LR-Decomposition
    for (int k = 0; k < this->dim(); k++)
    {
        i_max = std::min(this->dim() - 1, k + this->num_lower());
        for (int i = k + 1; i <= i_max; i++)
        {
            ROS_ASSERT(this->operator()(k, k) != 0.0);
            x = -this->operator()(i, k) / this->operator()(k, k);
            this->operator()(i, k) = -x;
            j_max = std::min(this->dim() - 1, k + this->num_upper());
            for (int j = k + 1; j <= j_max; j++)
            {
                this->operator()(i, j) += x * this->operator()(k, j);
            }
        }
    }
}

// solves Ly = b
std::vector<double> Cable::band_matrix::l_solve(const std::vector<double> &b) const
{
    ROS_ASSERT(this->dim() == (int)b.size());
    std::vector<double> x(this->dim());
    int j_start;
    double sum;

    for (int i = 0; i < this->dim(); i++)
    {
        sum = 0.0;
        j_start = std::max(0, i - this->num_lower());

        for (int j = j_start; j < i; j++)
        {
            sum += this->operator()(i, j) * x[j];
        }
        x[i] = (b[i] * this->saved_diag(i)) - sum;
    }
    return x;
}

// solver Rx = y
std::vector<double> Cable::band_matrix::r_solve(const std::vector<double> &b) const
{
    ROS_ASSERT(this->dim() == (int)b.size());
    std::vector<double> x(this->dim());
    int j_stop;
    double sum;

    for (int i = this->dim() - 1; i >= 0; i--)
    {
        sum = 0.0;
        j_stop = std::min(this->dim() - 1, i + this->num_upper());

        for (int j = i + 1; j <= j_stop; j++)
        {
            sum += this->operator()(i, j) * x[j];
        }
        x[i] = (b[i] - sum) / this->operator()(i, i);
    }
    return x;
}

// solves Ax = b
std::vector<double> Cable::band_matrix::lu_solve(const std::vector<double> &b, bool is_lu_decomposed)
{
    ROS_ASSERT(this->dim() == (int)b.size());
    std::vector<double> x, y;
    if (is_lu_decomposed == false)
    {
        this->lu_decompose();
    }
    y = this->l_solve(b);
    x = this->r_solve(y);

    return x;
}

// Machine precision of a double, i.e. the successor of 1 is 1+eps
double Cable::get_eps()
{
    // return std::numeric_limits<double>::epsilon();    // __DBL_EPSILON__
    return 2.2204460492503131e-16;
}

// Solutions for a+b*x = 0
std::vector<double> Cable::solve_linear(double a, double b)
{
    std::vector<double> x;
    if (b == 0.0)
    {
        if (a == 0.0)
        {
            x.resize(1);
            x[0] = 0.0;
            // infinite number of solutions
            return x;
        }
        else
        {
            // no solution
            return x;
        }
    }
    else
    {
        x.resize(1);
        x[0] = -a / b;
        return x;
    }
}

// solutions for a + b*x + c*x^2 = 0
std::vector<double> Cable::solve_quadratic(double a, double b, double c,
                                           int newton_iter = 0)
{
    if (c == 0.0)
    {
        return solve_linear(a, b); 
    }
    // rescale so that we solve x^2 + 2p x + q = (x+p)^2 + q - p^2 = 0
    double p = 0.5 * b / c;
    double q = a / c;
    double discr = p * p - q;
    const double eps = 0.5 * Cable::get_eps();
    double discr_err = (6.0 * (p * p) + 3.0 * fabs(q) + fabs(discr)) * eps;

    std::vector<double> x; // roots
    if (fabs(discr) <= discr_err)
    {
        // discriminant is zero --> one root
        x.resize(1);
        x[0] = -p;
    }
    else if (discr < 0)
    {
        // no root
    }
    else
    {
        // two roots
        x.resize(2);
        x[0] = -p - sqrt(discr);
        x[1] = -p + sqrt(discr);
    }

    // improve solution via newton steps
    for (size_t i = 0; i < x.size(); i++)
    {
        for (int k = 0; k < newton_iter; k++)
        {
            double f = (c * x[i] + b) * x[i] + a;
            double f1 = 2.0 * c * x[i] + b;
            // only adjust if slope is large enough
            if (fabs(f1) > 1e-8)
            {
                x[i] -= f / f1;
            }
        }
    }

    return x;
}

// solutions for the cubic equation: a + b*x +c*x^2 + d*x^3 = 0
// this is a naive implementation of the analytic solution without
// optimisation for speed or numerical accuracy
// newton_iter: number of newton iterations to improve analytical solution
// see also
//   gsl: gsl_poly_solve_cubic() in solve_cubic.c
//   octave: roots.m - via eigenvalues of the Frobenius companion matrix
std::vector<double> Cable::solve_cubic(double a, double b, double c, double d,
                                       int newton_iter = 0)
{
    if (d == 0.0)
    {
        return solve_quadratic(a, b, c, newton_iter);
    }

    // convert to normalised form: a + bx + cx^2 + x^3 = 0
    if (d != 1.0)
    {
        a /= d;
        b /= d;
        c /= d;
    }

    // convert to depressed cubic: z^3 - 3pz - 2q = 0
    // via substitution: z = x + c/3
    std::vector<double> z; // roots of the depressed cubic
    double p = -(1.0 / 3.0) * b + (1.0 / 9.0) * (c * c);
    double r = 2.0 * (c * c) - 9.0 * b;
    double q = -0.5 * a - (1.0 / 54.0) * (c * r);
    double discr = p * p * p - q * q; // discriminant
    // calculating numerical round-off errors with assumptions:
    //  - each operation is precise but each intermediate result x
    //    when stored has max error of x*eps
    //  - only multiplication with a power of 2 introduces no new error
    //  - a,b,c,d and some fractions (e.g. 1/3) have rounding errors eps
    //  - p_err << |p|, q_err << |q|, ... (this is violated in rare cases)
    // would be more elegant to use boost::numeric::interval<double>
    const double eps = Cable::get_eps();
    double p_err = eps * ((3.0 / 3.0) * fabs(b) + (4.0 / 9.0) * (c * c) + fabs(p));
    double r_err = eps * (6.0 * (c * c) + 18.0 * fabs(b) + fabs(r));
    double q_err = 0.5 * fabs(a) * eps + (1.0 / 54.0) * fabs(c) * (r_err + fabs(r) * 3.0 * eps) + fabs(q) * eps;
    double discr_err = (p * p) * (3.0 * p_err + fabs(p) * 2.0 * eps) + fabs(q) * (2.0 * q_err + fabs(q) * eps) + fabs(discr) * eps;

    // depending on the discriminant we get different solutions
    if (fabs(discr) <= discr_err)
    {
        // discriminant zero: one or two real roots
        if (fabs(p) <= p_err)
        {
            // p and q are zero: single root
            z.resize(1);
            z[0] = 0.0; // triple root
        }
        else
        {
            z.resize(2);
            z[0] = 2.0 * q / p; // single root
            z[1] = -0.5 * z[0]; // double root
        }
    }
    else if (discr > 0)
    {
        // three real roots: via trigonometric solution
        z.resize(3);
        double ac = (1.0 / 3.0) * acos(q / (p * sqrt(p)));
        double sq = 2.0 * sqrt(p);
        z[0] = sq * cos(ac);
        z[1] = sq * cos(ac - 2.0 * M_PI / 3.0);
        z[2] = sq * cos(ac - 4.0 * M_PI / 3.0);
    }
    else if (discr < 0.0)
    {
        // single real root: via Cardano's fromula
        z.resize(1);
        double sgnq = (q >= 0 ? 1 : -1);
        double basis = fabs(q) + sqrt(-discr);
        double C = sgnq * pow(basis, 1.0 / 3.0); // c++11 has std::cbrt()
        z[0] = C + p / C;
    }
    for (size_t i = 0; i < z.size(); i++)
    {
        // convert depressed cubic roots to original cubic: x = z - c/3
        z[i] -= (1.0 / 3.0) * c;
        // improve solution via newton steps
        for (int k = 0; k < newton_iter; k++)
        {
            double f = ((z[i] + c) * z[i] + b) * z[i] + a;
            double f1 = (3.0 * z[i] + 2.0 * c) * z[i] + b;
            // only adjust if slope is large enough
            if (fabs(f1) > 1e-8)
            {
                z[i] -= f / f1;
            }
        }
    }
    // ensure if a=0 we get exactly x=0 as root
    // TODO: remove this fudge
    if (a == 0.0)
    {
        ROS_ASSERT(z.size() > 0); // cubic should always have at least one root
        double xmin = fabs(z[0]);
        size_t imin = 0;
        for (size_t i = 1; i < z.size(); i++)
        {
            if (xmin > fabs(z[i]))
            {
                xmin = fabs(z[i]);
                imin = i;
            }
        }
        z[imin] = 0.0; // replace the smallest absolute value with 0
    }
    std::sort(z.begin(), z.end());
    return z;
}