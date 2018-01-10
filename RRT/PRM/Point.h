///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *  Contains code for 3D vectors.
 *  \file       Point.h
 *  \author     Thomas Horsch (derived from Pierre Terdiman)
 *  \date       Sept, 20, 2016
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef POINT_H
#define POINT_H
#include <algorithm>    // std::min
//#include "Box.h"

    // Forward declarations
    class HPoint;
    class Plane;
    class Matrix3x3;
    class Matrix4x4;
#define _X 0
#define _Y 1
#define _Z 2

    #define CROSS2D(a, b)   (a.x*b.y - b.x*a.y)

    const double EPSILON2 = 1.0e-20f;

    class Point
    {
        public:
        double          x, y, z;
        //! Empty constructor
		inline                  Point() { x = 0; y = 0; z = 0; }
        //! Constructor from a single double
//      inline_                 Point(double val) : x(val), y(val), z(val)                  {}
// Removed since it introduced the nasty "Point T = *Matrix4x4.GetTrans();" bug.......
        //! Constructor from doubles
        inline                  Point(double _x, double _y, double _z) : x(_x), y(_y), z(_z)    {}
        //! Constructor from array
        inline                  Point(const double f[3]) : x(f[_X]), y(f[_Y]), z(f[_Z])     {}
        //! Copy constructor
        inline                  Point(const Point& p) : x(p.x), y(p.y), z(p.z)              {}
        //! Destructor
        inline                  ~Point()                                                    {}

        //! Clears the vector
        inline  Point&          Zero()                                  { x =           y =         z = 0.0f;           return *this;   }

        //! + infinity
        //inline    Point&          SetPlusInfinity()                       { x =           y =         z = MAX_double;     return *this;   }
        ////! - infinity
        //inline    Point&          SetMinusInfinity()                      { x =           y =         z = MIN_double;     return *this;   }

        //! Sets positive unit random vector
//              Point&          PositiveUnitRandomVector();
        //! Sets unit random vector
//              Point&          UnitRandomVector();

        //! Assignment from values
        inline  Point&          Set(double _x, double _y, double _z)        { x  = _x;      y  = _y;    z  = _z;            return *this;   }
        //! Assignment from array
        inline  Point&          Set(const double f[3])                  { x  = f[_X];   y  = f[_Y]; z  = f[_Z];         return *this;   }
        //! Assignment from another point
        inline  Point&          Set(const Point& src)                   { x  = src.x;   y  = src.y; z  = src.z;         return *this;   }

        //! Adds a vector
        inline  Point&          Add(const Point& p)                     { x += p.x;     y += p.y;   z += p.z;           return *this;   }
        //! Adds a vector
        inline  Point&          Add(double _x, double _y, double _z)        { x += _x;      y += _y;    z += _z;            return *this;   }
        //! Adds a vector
        inline  Point&          Add(const double f[3])                  { x += f[_X];   y += f[_Y]; z += f[_Z];         return *this;   }
        //! Adds vectors
        inline  Point&          Add(const Point& p, const Point& q)     { x = p.x+q.x;  y = p.y+q.y;    z = p.z+q.z;    return *this;   }

        //! Subtracts a vector
        inline  Point&          Sub(const Point& p)                     { x -= p.x;     y -= p.y;   z -= p.z;           return *this;   }
        //! Subtracts a vector
        inline  Point&          Sub(double _x, double _y, double _z)        { x -= _x;      y -= _y;    z -= _z;            return *this;   }
        //! Subtracts a vector
        inline  Point&          Sub(const double f[3])                  { x -= f[_X];   y -= f[_Y]; z -= f[_Z];         return *this;   }
        //! Subtracts vectors
        inline  Point&          Sub(const Point& p, const Point& q)     { x = p.x-q.x;  y = p.y-q.y;    z = p.z-q.z;    return *this;   }

        //! this = -this
        inline  Point&          Neg()                                   { x = -x;       y = -y;         z = -z;         return *this;   }
        //! this = -a
        inline  Point&          Neg(const Point& a)                     { x = -a.x;     y = -a.y;       z = -a.z;       return *this;   }

        //! Multiplies by a scalar
        inline  Point&          Mult(double s)                          { x *= s;       y *= s;     z *= s;             return *this;   }

        //! this = a * scalar
        inline  Point&          Mult(const Point& a, double scalar)
                                {
                                    x = a.x * scalar;
                                    y = a.y * scalar;
                                    z = a.z * scalar;
                                    return *this;
                                }

        //! this = a + b * scalar
        inline  Point&          Mac(const Point& a, const Point& b, double scalar)
                                {
                                    x = a.x + b.x * scalar;
                                    y = a.y + b.y * scalar;
                                    z = a.z + b.z * scalar;
                                    return *this;
                                }

        //! this = this + a * scalar
        inline  Point&          Mac(const Point& a, double scalar)
                                {
                                    x += a.x * scalar;
                                    y += a.y * scalar;
                                    z += a.z * scalar;
                                    return *this;
                                }

        //! this = a - b * scalar
        inline  Point&          Msc(const Point& a, const Point& b, double scalar)
                                {
                                    x = a.x - b.x * scalar;
                                    y = a.y - b.y * scalar;
                                    z = a.z - b.z * scalar;
                                    return *this;
                                }

        //! this = this - a * scalar
        inline  Point&          Msc(const Point& a, double scalar)
                                {
                                    x -= a.x * scalar;
                                    y -= a.y * scalar;
                                    z -= a.z * scalar;
                                    return *this;
                                }

        //! this = a + b * scalarb + c * scalarc
        inline  Point&          Mac2(const Point& a, const Point& b, double scalarb, const Point& c, double scalarc)
                                {
                                    x = a.x + b.x * scalarb + c.x * scalarc;
                                    y = a.y + b.y * scalarb + c.y * scalarc;
                                    z = a.z + b.z * scalarb + c.z * scalarc;
                                    return *this;
                                }

        //! this = a - b * scalarb - c * scalarc
        inline  Point&          Msc2(const Point& a, const Point& b, double scalarb, const Point& c, double scalarc)
                                {
                                    x = a.x - b.x * scalarb - c.x * scalarc;
                                    y = a.y - b.y * scalarb - c.y * scalarc;
                                    z = a.z - b.z * scalarb - c.z * scalarc;
                                    return *this;
                                }

        //! this = mat * a
        inline  Point&          Mult(const Matrix3x3& mat, const Point& a);

        //! this = mat1 * a1 + mat2 * a2
        inline  Point&          Mult2(const Matrix3x3& mat1, const Point& a1, const Matrix3x3& mat2, const Point& a2);

        //! this = this + mat * a
        inline  Point&          Mac(const Matrix3x3& mat, const Point& a);

        //! this = transpose(mat) * a
        inline  Point&          TransMult(const Matrix3x3& mat, const Point& a);

        //! Linear interpolate between two vectors: this = a + t * (b - a)
        inline  Point&          Lerp(const Point& a, const Point& b, double t)
                                {
                                    x = a.x + t * (b.x - a.x);
                                    y = a.y + t * (b.y - a.y);
                                    z = a.z + t * (b.z - a.z);
                                    return *this;
                                }

        //! Hermite interpolate between p1 and p2. p0 and p3 are used for finding gradient at p1 and p2.
        //! this =  p0 * (2t^2 - t^3 - t)/2
        //!         + p1 * (3t^3 - 5t^2 + 2)/2
        //!         + p2 * (4t^2 - 3t^3 + t)/2
        //!         + p3 * (t^3 - t^2)/2
        inline  Point&          Herp(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double t)
                                {
                                    double t2 = t * t;
                                    double t3 = t2 * t;
                                    double kp0 = (2.0f * t2 - t3 - t) * 0.5f;
                                    double kp1 = (3.0f * t3 - 5.0f * t2 + 2.0f) * 0.5f;
                                    double kp2 = (4.0f * t2 - 3.0f * t3 + t) * 0.5f;
                                    double kp3 = (t3 - t2) * 0.5f;
                                    x = p0.x * kp0 + p1.x * kp1 + p2.x * kp2 + p3.x * kp3;
                                    y = p0.y * kp0 + p1.y * kp1 + p2.y * kp2 + p3.y * kp3;
                                    z = p0.z * kp0 + p1.z * kp1 + p2.z * kp2 + p3.z * kp3;
                                    return *this;
                                }

        //! this = rotpos * r + linpos
        inline  Point&          Transform(const Point& r, const Matrix3x3& rotpos, const Point& linpos);

        //! this = trans(rotpos) * (r - linpos)
        inline  Point&          InvTransform(const Point& r, const Matrix3x3& rotpos, const Point& linpos);

        //! Returns MIN(x, y, z);
//      inline  double          Min()               const       { return min(x, min(y, z));                                             }
        //! Returns MAX(x, y, z);
//      inline  double          Max()               const       { return max(x, max(y, z));                                             }
        //! Sets each element to be componentwise minimum
//      inline  Point&          Min(const Point& p)             { x = min(x, p.x); y = min(y, p.y); z = min(z, p.z);    return *this;   }
        //! Sets each element to be componentwise maximum
//      inline  Point&          Max(const Point& p)             { x = max(x, p.x); y = max(y, p.y); z = max(z, p.z);    return *this;   }

        //! Clamps each element
        inline  Point&          Clamp(double min, double max)
                                {
                                    if(x<min)   x=min;  if(x>max)   x=max;
                                    if(y<min)   y=min;  if(y>max)   y=max;
                                    if(z<min)   z=min;  if(z>max)   z=max;
                                    return *this;
                                }

        //! Computes square magnitude
        inline  double          SquareMagnitude()   const       { return x*x + y*y + z*z;                                               }
        //! Computes magnitude
        inline  double          Magnitude()         const       { return sqrt(x*x + y*y + z*z);                                     }
        //! Computes volume
        inline  double          Volume()            const       { return x * y * z;                                                     }

        ////! Checks the point is near zero
        //inline    bool            ApproxZero()        const       { return SquareMagnitude() < EPSILON2;                                  }

        ////! Tests for exact zero vector
        //inline    BOOL            IsZero()            const
        //                      {
        //                          if(IR(x) || IR(y) || IR(z)) return FALSE;
        //                          return TRUE;
        //                      }

        ////! Checks point validity
        //inline    BOOL            IsValid()           const
        //                      {
        //                          if(!IsValiddouble(x))   return FALSE;
        //                          if(!IsValiddouble(y))   return FALSE;
        //                          if(!IsValiddouble(z))   return FALSE;
        //                          return TRUE;
        //                      }

        //! Slighty moves the point
        //      void            Tweak(udword coord_mask, udword tweak_mask)
        //                      {
        //                          if(coord_mask&1)    { udword Dummy = IR(x); Dummy^=tweak_mask;  x = FR(Dummy); }
        //                          if(coord_mask&2)    { udword Dummy = IR(y); Dummy^=tweak_mask;  y = FR(Dummy); }
        //                          if(coord_mask&4)    { udword Dummy = IR(z); Dummy^=tweak_mask;  z = FR(Dummy); }
        //                      }

        //#define TWEAKMASK     0x3fffff
        //#define TWEAKNOTMASK  ~TWEAKMASK
        ////! Slighty moves the point out
        //inline    void            TweakBigger()
        //                      {
        //                          udword  Dummy = (IR(x)&TWEAKNOTMASK);   if(!IS_NEGATIVE_double(x))  Dummy+=TWEAKMASK+1; x = FR(Dummy);
        //                                  Dummy = (IR(y)&TWEAKNOTMASK);   if(!IS_NEGATIVE_double(y))  Dummy+=TWEAKMASK+1; y = FR(Dummy);
        //                                  Dummy = (IR(z)&TWEAKNOTMASK);   if(!IS_NEGATIVE_double(z))  Dummy+=TWEAKMASK+1; z = FR(Dummy);
        //                      }

        ////! Slighty moves the point in
        //inline    void            TweakSmaller()
        //                      {
        //                          udword  Dummy = (IR(x)&TWEAKNOTMASK);   if(IS_NEGATIVE_double(x))   Dummy+=TWEAKMASK+1; x = FR(Dummy);
        //                                  Dummy = (IR(y)&TWEAKNOTMASK);   if(IS_NEGATIVE_double(y))   Dummy+=TWEAKMASK+1; y = FR(Dummy);
        //                                  Dummy = (IR(z)&TWEAKNOTMASK);   if(IS_NEGATIVE_double(z))   Dummy+=TWEAKMASK+1; z = FR(Dummy);
        //                      }

        //! Normalizes the vector
        inline  Point&          Normalize()
                                {
                                    double M = x*x + y*y + z*z;
                                    if(M)
                                    {
                                        M = 1.0 / sqrt(M);
                                        x *= M;
                                        y *= M;
                                        z *= M;
                                    }
                                    return *this;
                                }

        //! Sets vector length
        inline  Point&          SetLength(double length)
                                {
                                    double NewLength = length / Magnitude();
                                    x *= NewLength;
                                    y *= NewLength;
                                    z *= NewLength;
                                    return *this;
                                }

        //! Clamps vector length
        inline  Point&          ClampLength(double limit_length)
                                {
                                    if(limit_length>=0.0f)  // Magnitude must be positive
                                    {
                                        double CurrentSquareLength = SquareMagnitude();

                                        if(CurrentSquareLength > limit_length * limit_length)
                                        {
                                            double Coeff = limit_length / sqrt(CurrentSquareLength);
                                            x *= Coeff;
                                            y *= Coeff;
                                            z *= Coeff;
                                        }
                                    }
                                    return *this;
                                }

        //! Computes distance to another point
        inline  double          Distance(const Point& b)            const
                                {
                                    return sqrt((x - b.x)*(x - b.x) + (y - b.y)*(y - b.y) + (z - b.z)*(z - b.z));
                                }

        //! Computes square distance to another point
        inline  double          SquareDistance(const Point& b)      const
                                {
                                    return ((x - b.x)*(x - b.x) + (y - b.y)*(y - b.y) + (z - b.z)*(z - b.z));
                                }

        //! Dot product dp = this|a
        inline  double          Dot(const Point& p)                 const       {   return p.x * x + p.y * y + p.z * z;             }

        //! Cross product this = a x b
        inline  Point&          Cross(const Point& a, const Point& b)
                                {
                                    x = a.y * b.z - a.z * b.y;
                                    y = a.z * b.x - a.x * b.z;
                                    z = a.x * b.y - a.y * b.x;
                                    return *this;
                                }

        //! Vector code ( bitmask = sign(z) | sign(y) | sign(x) )
        //inline    udword          VectorCode()                        const
        //                      {
        //                          return (IR(x)>>31) | ((IR(y)&SIGN_BITMASK)>>30) | ((IR(z)&SIGN_BITMASK)>>29);
        //                      }

        ////! Returns largest axis
        //inline    PointComponent  LargestAxis()                       const
        //                      {
        //                          const double* Vals = &x;
        //                          PointComponent m = _X;
        //                          if(Vals[_Y] > Vals[m]) m = _Y;
        //                          if(Vals[_Z] > Vals[m]) m = _Z;
        //                          return m;
        //                      }

        //! Returns closest axis
        //inline    PointComponent  ClosestAxis()                       const
        //                      {
        //                          const double* Vals = &x;
        //                          PointComponent m = _X;
        //                          if(AIR(Vals[_Y]) > AIR(Vals[m])) m = _Y;
        //                          if(AIR(Vals[_Z]) > AIR(Vals[m])) m = _Z;
        //                          return m;
        //                      }

        //! Returns smallest axis
        //inline    PointComponent  SmallestAxis()                      const
        //                      {
        //                          const double* Vals = &x;
        //                          PointComponent m = _X;
        //                          if(Vals[_Y] < Vals[m]) m = _Y;
        //                          if(Vals[_Z] < Vals[m]) m = _Z;
        //                          return m;
        //                      }

        //! Refracts the point
        //      Point&          Refract(const Point& eye, const Point& n, double refractindex, Point& refracted);

        ////! Projects the point onto a plane
        //      Point&          ProjectToPlane(const Plane& p);

        ////! Projects the point onto the screen
        //      void            ProjectToScreen(double halfrenderwidth, double halfrenderheight, const Matrix4x4& mat, HPoint& projected) const;

        ////! Unfolds the point onto a plane according to edge(a,b)
        //      Point&          Unfold(Plane& p, Point& a, Point& b);

        //! Hash function from Ville Miettinen
        //inline    udword          GetHashValue()                      const
        //                      {
        //                          const udword* h = (const udword*)(this);
        //                          udword f = (h[0]+h[1]*11-(h[2]*17)) & 0x7fffffff;   // avoid problems with +-0
        //                          return (f>>22)^(f>>12)^(f);
        //                      }

        //! Stuff magic values in the point, marking it as explicitely not used.
                void            SetNotUsed();
        //! Checks the point is marked as not used
                bool            IsNotUsed()                         const;

        // Arithmetic operators

        //! Unary operator for Point Negate = - Point
        inline  Point           operator-()                         const       { return Point(-x, -y, -z);                         }

        //! Operator for Point Plus = Point + Point.
        inline  Point           operator+(const Point& p)           const       { return Point(x + p.x, y + p.y, z + p.z);          }
        //! Operator for Point Minus = Point - Point.
        inline  Point           operator-(const Point& p)           const       { return Point(x - p.x, y - p.y, z - p.z);          }

        //! Operator for Point Mul   = Point * Point.
        inline  Point           operator*(const Point& p)           const       { return Point(x * p.x, y * p.y, z * p.z);          }
        //! Operator for Point Scale = Point * double.
        inline  Point           operator*(double s)                 const       { return Point(x * s,   y * s,   z * s );           }
        //! Operator for Point Scale = double * Point.
        inline friend   Point   operator*(double s, const Point& p)             { return Point(s * p.x, s * p.y, s * p.z);          }

        //! Operator for Point Div   = Point / Point.
        inline  Point           operator/(const Point& p)           const       { return Point(x / p.x, y / p.y, z / p.z);          }
        //! Operator for Point Scale = Point / double.
        inline  Point           operator/(double s)                 const       { s = 1.0f / s; return Point(x * s, y * s, z * s);  }
        //! Operator for Point Scale = double / Point.
        inline  friend  Point   operator/(double s, const Point& p)             { return Point(s / p.x, s / p.y, s / p.z);          }

        //! Operator for double DotProd = Point | Point.
        inline  double          operator|(const Point& p)           const       { return x*p.x + y*p.y + z*p.z;                     }
        //! Operator for Point VecProd = Point ^ Point.
        inline  Point           operator^(const Point& p)           const
                                {
                                    return Point(
                                    y * p.z - z * p.y,
                                    z * p.x - x * p.z,
                                    x * p.y - y * p.x );
                                }

        //! Operator for Point += Point.
        inline  Point&          operator+=(const Point& p)                      { x += p.x; y += p.y; z += p.z; return *this;       }
        //! Operator for Point += double.
        inline  Point&          operator+=(double s)                                { x += s;   y += s;   z += s;   return *this;       }

        //! Operator for Point -= Point.
        inline  Point&          operator-=(const Point& p)                      { x -= p.x; y -= p.y; z -= p.z; return *this;       }
        //! Operator for Point -= double.
        inline  Point&          operator-=(double s)                                { x -= s;   y -= s;   z -= s;   return *this;       }

        //! Operator for Point *= Point.
        inline  Point&          operator*=(const Point& p)                      { x *= p.x; y *= p.y; z *= p.z; return *this;       }
        //! Operator for Point *= double.
        inline  Point&          operator*=(double s)                                { x *= s; y *= s; z *= s;       return *this;       }

        //! Operator for Point /= Point.
        inline  Point&          operator/=(const Point& p)                      { x /= p.x; y /= p.y; z /= p.z; return *this;       }
        //! Operator for Point /= double.
        inline  Point&          operator/=(double s)                                { s = 1.0f/s; x *= s; y *= s; z *= s; return *this; }

        // Logical operators

        //! Operator for "if(Point==Point)"
        //inline    bool            operator==(const Point& p)          const       { return ( (IR(x)==IR(p.x))&&(IR(y)==IR(p.y))&&(IR(z)==IR(p.z)));   }
        ////! Operator for "if(Point!=Point)"
        //inline    bool            operator!=(const Point& p)          const       { return ( (IR(x)!=IR(p.x))||(IR(y)!=IR(p.y))||(IR(z)!=IR(p.z)));   }

        // Arithmetic operators

        //! Operator for Point Mul = Point * Matrix3x3.
        inline  Point           operator*(const Matrix3x3& mat)     const
                                {
                                    class ShadowMatrix3x3{ public: double m[3][3]; };   // To allow inlining
                                    const ShadowMatrix3x3* Mat = (const ShadowMatrix3x3*)&mat;

                                    return Point(
                                    x * Mat->m[0][0] + y * Mat->m[1][0] + z * Mat->m[2][0],
                                    x * Mat->m[0][1] + y * Mat->m[1][1] + z * Mat->m[2][1],
                                    x * Mat->m[0][2] + y * Mat->m[1][2] + z * Mat->m[2][2] );
                                }

        //! Operator for Point Mul = Point * Matrix4x4.
        inline  Point           operator*(const Matrix4x4& mat)     const
                                {
                                    class ShadowMatrix4x4{ public: double m[4][4]; };   // To allow inlining
                                    const ShadowMatrix4x4* Mat = (const ShadowMatrix4x4*)&mat;

                                    return Point(
                                    x * Mat->m[0][0] + y * Mat->m[1][0] + z * Mat->m[2][0] + Mat->m[3][0],
                                    x * Mat->m[0][1] + y * Mat->m[1][1] + z * Mat->m[2][1] + Mat->m[3][1],
                                    x * Mat->m[0][2] + y * Mat->m[1][2] + z * Mat->m[2][2] + Mat->m[3][2]);
                                }

        //! Operator for Point *= Matrix3x3.
        inline  Point&          operator*=(const Matrix3x3& mat)
                                {
                                    class ShadowMatrix3x3{ public: double m[3][3]; };   // To allow inlining
                                    const ShadowMatrix3x3* Mat = (const ShadowMatrix3x3*)&mat;

                                    double xp = x * Mat->m[0][0] + y * Mat->m[1][0] + z * Mat->m[2][0];
                                    double yp = x * Mat->m[0][1] + y * Mat->m[1][1] + z * Mat->m[2][1];
                                    double zp = x * Mat->m[0][2] + y * Mat->m[1][2] + z * Mat->m[2][2];

                                    x = xp; y = yp; z = zp;

                                    return *this;
                                }

        // dist_Point_to_Segment(): get the distance of a point to a segment
        //     Input:  a Point P and a Segment S (in any dimension) - S from P0 to P1 -
        //     Return: the shortest distance from P to S
        double dist_Point_to_Segment(Point P,Point P0, Point P1) {
            Point v = P1 - P0;
            Point w = P - P0;

            double c1 = w | v; // scalar product
            if (c1 <= 0)
                return P.Distance(P0);

            double c2 = v | v; // scalar product
            if (c2 <= c1)
                return P.Distance(P1);

            double b = c1 / c2;
            Point Pb = P0 + b * v;
            return P.Distance(Pb);
        }

		bool equals(Point a) {
			if (a.x == x && a.y == y && a.z == z) {
				return true;
			}
			return false;
		}

//      private:

    };

    //FUNCTION ICEMATHS_API void Normalize1(Point& a);
    //FUNCTION ICEMATHS_API void Normalize2(Point& a);

#endif //POINT_
