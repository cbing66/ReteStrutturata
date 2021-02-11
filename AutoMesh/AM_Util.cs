/*
MIT License

Copyright (c) 2005-2020 Alessandro Corazzin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoMesh
{
  class AM_Util
  {
    // Math values
    internal const double FLT_EPSILON = 1.192092896e-07;
    internal const double DBL_EPSILON = 2.2204460492503131e-016;


    internal static bool IsEqual(double a, double b, double tol = FLT_EPSILON)
    {
      return Math.Abs(a - b)<tol;
    }

    internal static Vector2d NormalVersor(Vector2d v, bool ccw = true)
    {
      v.Unitize();
      return ccw ?
             new Vector2d(-v.Y, v.X) :
             new Vector2d(v.Y, -v.X);
    }

    internal static double TriArea(Point2d a, Point2d b, Point2d c)
    {
      return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
    }

    internal static bool InCircle(Point2d a, Point2d b, Point2d c, Point2d d)
    {
      return (a.X * a.X + a.Y * a.Y) * TriArea(b, c, d) -
             (b.X * b.X + b.Y * b.Y) * TriArea(a, c, d) +
             (c.X * c.X + c.Y * c.Y) * TriArea(a, b, d) -
             (d.X * d.X + d.Y * d.Y) * TriArea(a, b, c) > 0;
    }

    internal static void InCircle(Point2d P1, Point2d P2, Point2d P3, ref Point2d inCenter, ref double inRadius)
    {
      Vector2d v1 = P2 - P1;
      Vector2d v2 = P3 - P1;
      Vector2d v3 = P3 - P2;

      v1.Unitize();
      v2.Unitize();
      v3.Unitize();

      Vector2d b1 = 0.5 * (v1 + v2);
      Vector2d b2 = 0.5 * (v3 - v2);

      Point3d p1g = new Point3d(P1.X, P1.Y, 0);
      Point3d p2g = new Point3d(P2.X, P2.Y, 0);

      Line a = new Line(p1g, new Vector3d(b1.X, b1.Y, 0));
      Line b = new Line(p2g, new Vector3d(b2.X, b2.Y, 0));

      double par_a, par_b;
      if (Intersection.LineLine(a, b, out par_a, out par_b)) {
        var center = a.PointAt(par_a);

        Point3d ortho = new Line(p1g, p2g).ClosestPoint(center, false);

        inRadius = (center - ortho).Length;
        inCenter = new Point2d(center);
      } else {
        // caso degenere
        if ((P1- P2).Length < FLT_EPSILON) {
          inRadius = 0;
          if (P1 == P2)
            inCenter = 0.5 * (P2 + P3);
          else
            inCenter = 0.5 * (P1 + P2);
        }
      }

    }

    internal static Point2d To2d(Point3d p)
    {
      return new Point2d(p.X, p.Y);
    }

    internal static Vector2d To2d(Vector3d v)
    {
      return new Vector2d(v.X, v.Y);
    }

    internal static Point3d To3d(Point2d p)
    {
      return new Point3d(p.X, p.Y, 0);
    }

    internal static Vector3d To3d(Vector2d v)
    {
      return new Vector3d(v.X, v.Y, 0);
    }

    internal static bool CCW(Point2d a, Point2d b, Point2d c)
    {
      return (TriArea(a, b, c) > 0);
    }

    internal static int IntNear(double a)
    {
      return (a - (int)a) > 0.5 ? ((int)a + 1) : (int)a;
    }

    internal static void CircumCircle(Point2d P1, Point2d P2, Point2d P3,
                                      ref Point2d CircumCenter, ref double CircumRadius)
    {
      Point2d m = 0.5 * (P1 + P2);
      Point2d n = 0.5 * (P2 + P3);

      double num = (m.X - n.X) * (P2.X - P1.X) + (m.Y - n.Y) * (P2.Y - P1.Y);
      double den = (P2.X - P1.X) * (P3.Y - P2.Y) - (P3.X - P2.X) * (P2.Y - P1.Y);

      if (den != 0) {
        double ratio = num / den;

        CircumCenter.X = n.X + ratio* (P3.Y - P2.Y);
        CircumCenter.Y = n.Y - ratio* (P3.X - P2.X);
        CircumRadius = (CircumCenter - P1).Length;
      } else {
        // Triangolo degenere
        if (P1 == P2)
          CircumCenter = n;
        else
          CircumCenter = m;

        CircumRadius = (CircumCenter - P1).Length;
      }
    }

    internal static bool IsInside(Vector2d v1, Vector2d v2, Vector2d v)
    {
      double ang = Math.Atan2(v.Y, v.X);
      double ang1 = Math.Atan2(v1.Y, v1.X);
      double ang2 = Math.Atan2(v2.Y, v2.X);

      while (ang2<ang1)
        ang2 += 2*Math.PI;

      double addAngle = 0;
      while (ang + addAngle<ang1)
        addAngle += 2*Math.PI;

      if (ang1<ang + addAngle && ang + addAngle<ang2) {
        return true;
      }
      return false;
    }

    internal static bool CheckSwapEdge(AM_Edge pedge)
    {
      // Controlla l'ammissibilità dello swap.
      // Uno spigolo può essere scambiato all'interno di un quadrangolo
      // se quest'ultimo è convesso.

      Point2d p1 = pedge.CcwEdge().DestCoord();
      Point2d p2 = pedge.OrgCoord();
      Point2d p3 = pedge.DestCoord();
      Point2d p4 = pedge.Symm().CcwEdge().DestCoord();

      Vector2d v1 = p4-p2;
      Vector2d v2 = p1-p2;

      v1.Unitize();
      v2.Unitize();

      if ((v1.X * v2.Y - v2.X * v1.Y) < FLT_EPSILON)
        return false;

      Vector2d v3 = p1-p3;
      Vector2d v4 = p4-p3;

      v3.Unitize();
      v4.Unitize();

      if ((v3.X * v4.Y - v4.X * v3.Y) < FLT_EPSILON)
        return false;

      return true;
    }

    internal static double [,] AffineMatrix(Point2d org, Vector2d dir)
    {
      // Matrice [2,4] per descrivere la matrice di trasformazione affine con base ortogonale
      // e i dati per della colonna 3 per la sua inversa
      var m = new double[2, 4];

      m[0,2] = org.X;
      m[1,2] = org.Y;

      dir.Unitize();

      m[0,0] = dir.X;
      m[1,0] = dir.Y;
      m[0,1] = -dir.Y;
      m[1,1] = dir.X;

      m[0, 3] = -(m[0, 0] * org.X + m[1, 0] * org.Y);
      m[1, 3] = -(m[0, 1] * org.X + m[1, 1] * org.Y);

      return m;
    }

    internal static Point2d ToGlobal(double [,] m, Point2d locPt)
    {
      return new Point2d(
               m[0,0] * locPt.X + m[0,1] * locPt.Y + m[0,2],
               m[1,0] * locPt.X + m[1,1] * locPt.Y + m[1,2]);
    }

    internal static Point2d ToLocal(double[,] m, Point2d p)
    {
      return new Point2d(
               m[0, 0] * p.X + m[1, 0] * p.Y + m[0, 3],
               m[0, 1] * p.X + m[1, 1] * p.Y + m[0, 4]);
    }

    internal enum EInterpolation {
      Linear,
      Parabolic,
      //Quadratic,
      //Cubic,
    }

    internal static double Interpolation(EInterpolation interpolation, double start, double end, double r)
    {
      if (r <= 0)
        return start;
      if (r >= 1)
        return end;

      double v = start;
      double dv = end - start;

      switch (interpolation) {
      case EInterpolation.Linear:
        v = start + (end - start) * r;
        break;

      case EInterpolation.Parabolic:
        v = start + (r * dv * (2 - r));
        break;
      }

      return v;
    }

  }
}
